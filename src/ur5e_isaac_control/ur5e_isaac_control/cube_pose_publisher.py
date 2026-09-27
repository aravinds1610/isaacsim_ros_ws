#!/usr/bin/env python3
"""Publish the AprilTag-marked cube's pose in the robot's ``base_link`` frame.

Subscribes to Isaac Sim's ``/rgb`` + ``/camera_info``, detects the 36h11 tag,
and publishes where the cube is in coordinates MoveIt can plan to. Replaces the
one-shot detection scripts used while bringing the perception chain up.

The transform chain, and why each piece is here:

    T_base_cube = T_base_optical . T_optical_tag . (tag -> cube offset)

``apriltag_pose.py`` supplies ``T_optical_tag`` in the OpenCV optical frame
(X right, Y down, Z out of the lens). ``T_base_optical`` is the extrinsic: a
*constant*, because the camera is bolted to the world rather than the wrist.
Measure it with ``scripts/isaac_camera_extrinsic.py``, which already folds in
the USD-camera -> ROS-optical 180-degree flip.

Two deliberate departures from "just use what the detector says", both measured
on 2026-08-11 (see docs/PLAN.md):

* **Depth comes from known geometry, not vision** -- applied by sliding along
  the camera ray, not by overwriting Z. A monocular camera infers depth purely
  from apparent size, and mipmap blur makes the detector read the tag ~1.1 px
  small, so range comes out 1-2% long (28-48 mm here). The *direction* to the
  tag is reliable; only the distance along it is biased. So given the tag's
  known height (``tag_z_base``), slide along the ray until it reaches that
  plane.

  Overwriting Z alone would not be enough: lateral position is computed as
  ``(u - cx)/fx * Z``, i.e. proportional to the depth estimate, so a depth bias
  leaks sideways in proportion to how far off-axis the tag is. Measured
  2026-08-11 with the tag 7 degrees off-axis, raw error was +4.5 mm in X,
  -5.1 mm in Y, -55.5 mm in Z; after the ray-plane correction, 0.25 mm and
  0.33 mm laterally and exact in Z. On-axis the sideways part vanishes, which
  is why an earlier on-axis test showed 0.05 mm and hid the effect.

  Leave ``tag_z_base`` unset (NaN) to fall back to raw vision, with a warning.

* **Roll and pitch are forced to zero; only yaw is kept.** Viewing a flat square
  nearly head-on makes out-of-plane tilt degenerate -- two mirror solutions fit
  almost equally well and the estimate flickers. The cube lies flat on the belt,
  so the tilt is known to be zero. In-plane yaw stays well determined.

Publishes:
    /cube_pose    geometry_msgs/PoseStamped   in base_link
    /cube_marker  visualization_msgs/Marker   for RViz
    TF            base_link -> <camera_frame> (static, from the extrinsic)
"""
import math

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster
from ur5e_isaac_control.apriltag_pose import AprilTagDetector
from visualization_msgs.msg import Marker


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class CubePosePublisher(Node):

    def __init__(self):
        super().__init__('cube_pose_publisher')

        # --- tag / cube geometry -----------------------------------------
        # tag_size is the BLACK SQUARE's edge, not the printed image's. The
        # PNGs are 1040 px with an 800 px black square, so a decal quad of edge
        # q carries a black square of q * 800/1040. Overstating it scales every
        # reported distance by the same factor.
        self.declare_parameter('tag_size', 0.03692)
        self.declare_parameter('tag_id', 0)
        # The decal sits on the cube's TOP face, so the detected pose is a plane
        # this far above the cube's centre.
        self.declare_parameter('tag_to_cube_z', 0.026)
        self.declare_parameter('cube_size', 0.05)

        # --- extrinsic: base_link -> camera optical frame -----------------
        # From scripts/isaac_camera_extrinsic.py. Re-run it whenever the camera
        # OR the robot base moves -- it is the relationship between them.
        # Re-measured 2026-09-27 after the camera was raised to world
        # (14.4201, 2.59795, 4.89319) for a wider view; rotation unchanged.
        self.declare_parameter('extrinsic_xyz',
                               [-0.345118, 0.863705, 2.883689])
        self.declare_parameter('extrinsic_quat',
                               [0.707112, -0.707101, 0.0, -0.000011])

        # --- the known-geometry depth correction --------------------------
        # Height of the TAG plane in base_link, measured off the USD stage as
        #   cube_world_z + tag_to_cube_z - base_world_z
        # NaN means "trust vision", which carries the range bias described in
        # the module docstring. Re-measure whenever the belt or the robot base
        # changes height.
        self.declare_parameter('tag_z_base', -0.177978)

        # A cube looks identical every 90 degrees, so any of four grasp angles
        # is equally good. Folding yaw into [-45, 45] keeps the wrist near zero.
        self.declare_parameter('normalize_yaw_90', True)

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'sim_camera')
        self.declare_parameter('image_topic', '/rgb')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('publish_static_tf', True)

        g = self.get_parameter
        self.tag_size = g('tag_size').value
        self.tag_id = g('tag_id').value
        self.tag_to_cube_z = g('tag_to_cube_z').value
        self.cube_size = g('cube_size').value
        self.tag_z_base = g('tag_z_base').value
        self.normalize_yaw = g('normalize_yaw_90').value
        self.base_frame = g('base_frame').value
        self.camera_frame = g('camera_frame').value

        ext_t = list(g('extrinsic_xyz').value)
        ext_q = list(g('extrinsic_quat').value)
        self.ext_t = np.array(ext_t, dtype=float)
        self.ext_R = quat_to_matrix(*ext_q)

        self.detector = AprilTagDetector(self.tag_size)
        self.camera_info = None
        self._warned_no_info = False
        self._detections = 0

        self.pose_pub = self.create_publisher(PoseStamped, '/cube_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/cube_marker', 10)

        self.create_subscription(CameraInfo, g('camera_info_topic').value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, g('image_topic').value,
                                 self._on_image, qos_profile_sensor_data)

        if g('publish_static_tf').value:
            self._publish_static_tf(ext_t, ext_q)

        z_note = ('raw vision (NOT recommended: 1-2% range bias)'
                  if math.isnan(self.tag_z_base)
                  else f'ray-plane onto tag_z_base={self.tag_z_base:+.6f} m')
        self.get_logger().info(
            f'cube_pose_publisher up. tag_size={self.tag_size} m, '
            f'tag_id={self.tag_id}, Z from {z_note}')

    def _publish_static_tf(self, t, q):
        """Broadcast the extrinsic so RViz and anything else agree with us."""
        self._static_tf = StaticTransformBroadcaster(self)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.child_frame_id = self.camera_frame
        msg.transform.translation.x, msg.transform.translation.y, \
            msg.transform.translation.z = t
        msg.transform.rotation.x, msg.transform.rotation.y, \
            msg.transform.rotation.z, msg.transform.rotation.w = q
        self._static_tf.sendTransform(msg)

    def _on_info(self, msg):
        self.camera_info = msg

    def _on_image(self, msg):
        if self.camera_info is None:
            if not self._warned_no_info:
                self.get_logger().warn(
                    'images arriving but no /camera_info yet — cannot compute '
                    'a pose without the intrinsics')
                self._warned_no_info = True
            return

        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().error(
                f'unsupported encoding {msg.encoding!r}; expected rgb8/bgr8')
            return

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1]           # to BGR, which OpenCV expects

        K = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        D = np.array(self.camera_info.d, dtype=np.float64)
        detections = self.detector.detect(np.ascontiguousarray(img), K, D)

        hit = next((d for d in detections if d['id'] == self.tag_id), None)
        if hit is None:
            self.get_logger().warn(
                f'no tag id {self.tag_id} in view', throttle_duration_sec=5.0)
            return

        self._publish(hit, msg.header.stamp)

    def _publish(self, det, stamp):
        # Tag position and orientation in base_link.
        p_tag = self.ext_R @ np.asarray(det['tvec'], dtype=float) + self.ext_t
        R_tag = self.ext_R @ det['T_cam_tag'][:3, :3]

        # Yaw only: the cube lies flat, so roll/pitch are known to be zero and
        # the vision estimate for them is degenerate anyway.
        yaw = math.atan2(R_tag[1, 0], R_tag[0, 0])
        if self.normalize_yaw:
            yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4

        # Depth from known geometry, applied along the camera ray so that X and
        # Y are corrected too (see module docstring). self.ext_t is the camera's
        # origin in base_link, so p_tag - ext_t is the direction to the tag.
        if not math.isnan(self.tag_z_base):
            ray = p_tag - self.ext_t
            if abs(ray[2]) > 1e-6:
                scale = (self.tag_z_base - self.ext_t[2]) / ray[2]
                p_tag = self.ext_t + scale * ray

        cube = np.array([p_tag[0], p_tag[1], p_tag[2] - self.tag_to_cube_z])

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(cube[0])
        pose.pose.position.y = float(cube[1])
        pose.pose.position.z = float(cube[2])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns = 'cube'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = self.cube_size
        marker.color.r, marker.color.g, marker.color.b = 0.1, 0.9, 0.2
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

        self._detections += 1
        if self._detections == 1 or self._detections % 100 == 0:
            self.get_logger().info(
                f'cube @ ({cube[0]:+.4f}, {cube[1]:+.4f}, {cube[2]:+.4f}) '
                f'yaw {math.degrees(yaw):+.1f} deg, '
                f'{np.linalg.norm(cube):.3f} m from base '
                f'[{self._detections} detections]')


def main():
    rclpy.init()
    node = CubePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
