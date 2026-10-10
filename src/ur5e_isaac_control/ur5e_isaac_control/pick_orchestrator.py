#!/usr/bin/env python3
"""Fixed-point pick-and-place state machine (M4), with an EMPTY planning scene.

Drives the full chain that M0-M2 built: read the cube's pose from perception,
plan to it with MoveIt, execute through trajectory_bridge into Isaac Sim, close
the gripper, carry the cube to the bin, release, go home.

NO COLLISION OBJECTS ARE REGISTERED. This is deliberate, not an oversight.
M3 (planning-scene furniture) is skipped for now because:

  * Self-collision checking is unaffected -- it comes from the SRDF's collision
    matrix, not from the world, so the arm still cannot fold into itself.
  * An empty world is the EASIEST case for the planner. Every collision object
    is one more way for OMPL to fail, so this maximises the chance of a first
    successful pick.
  * Measuring the scene (scripts/isaac_scene_measure.py, 2026-08-11) showed the
    obvious version of M3 would have BLOCKED this milestone: the conveyor's
    bounding box spans x [-0.768, 3.235], y [+0.165, +1.341], z [-2.010, +0.301]
    in base_link, and the cube at (-0.365, +0.616, -0.204) sits inside it on all
    three axes. Registering the conveyor as one box makes every grasp pose
    permanently in collision. When furniture is added later it must stop at the
    belt surface rather than use the full AABB.

The consequence to be aware of: the arm CAN and may plan straight through the
conveyor, the table and the bin. In simulation that costs nothing -- PhysX will
interpenetrate and no hardware is at risk -- but do not port this to a real arm
without doing M3 first.

WHY THE MoveGroup ACTION AND NOT MoveGroupInterface: this install has no
moveit_py bindings (`import moveit` fails; only moveit_msgs is present), so the
Python convenience wrapper PLAN.md assumed is unavailable. Sending
moveit_msgs/action/MoveGroup to /move_action is what that wrapper does
internally, and it keeps this node dependency-free.

WHY THE GRASP ORIENTATION IS WHAT IT IS: grasp_tcp's +Z is the approach axis
(it inherits tool0's convention), and the cube is picked from directly above, so
+Z must point along base_link's -Z. That is a 180-degree rotation about X,
composed with the cube's yaw about base Z:

    R = Rz(yaw) . Rx(pi)   ->   quat (x, y, z, w) = (cos(yaw/2), sin(yaw/2), 0, 0)

At yaw = 0 this is (1, 0, 0, 0), a clean flip. cube_pose_publisher already folds
yaw into +/-45 degrees, so the wrist never takes the long way round.

WHY THE CUBE POSE IS FROZEN AT PICK TIME: once the gripper is over the cube it
occludes the tag, and once the cube is grasped its pose is defined by the
gripper, not by vision. So the sequence latches one pose (checked for freshness)
and stops listening. M5 is where a live pose stream starts mattering.

Sequence: home -> above cube -> down to grasp -> close -> lift -> above bin ->
release -> retreat -> home.
"""
import math
import threading
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# A "ready" pose, NOT the SRDF's straight-up "home". Straight up is a
# singularity: IK seeded from it lands on an arbitrary branch, and Pilz PTP
# then swept the upper arm through the base to get there. This pose is elbow
# up, TCP pointing straight down, panned toward the cube (atan2(0.616, -0.365)).
# Offline IK from here to the approach pose stays on the same branch (every
# joint moves < 0.9 rad) and matches the config the arm reached live.
HOME_JOINTS = [2.1, -1.5707, 1.5707, -1.5707, -1.5707, 0.0]

MOVEIT_SUCCESS = 1          # moveit_msgs/MoveItErrorCodes.SUCCESS


def grasp_quaternion(yaw):
    """Top-down approach at `yaw` about base Z, as (x, y, z, w).

    See the module docstring: Rz(yaw) . Rx(pi).
    """
    return (math.cos(yaw / 2.0), math.sin(yaw / 2.0), 0.0, 0.0)


def yaw_from_quaternion(x, y, z, w):
    """Yaw about Z from a quaternion, for reading the cube's orientation back."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PickOrchestrator(Node):

    def __init__(self):
        super().__init__('pick_orchestrator')

        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('tcp_link', 'grasp_tcp')
        # Distance from tcp_link to the fingertips along the approach axis.
        # 0 for our URDF (grasp_tcp IS the fingertips); 0.11181 when planning
        # with NVIDIA's bare-arm model, whose tool frame is tool0.
        self.declare_parameter('tcp_offset_z', 0.0)
        self.declare_parameter('base_frame', 'base_link')

        # --- grasp geometry ------------------------------------------------
        # grasp_tcp is the fingertip LINK ORIGIN at the closed knuckle angle,
        # not the pad contact face (see ur5e_gripper.urdf.xacro). At 0.0 (TCP at
        # the cube centre, 25 mm above the belt) the pads, which reach roughly
        # 30-40 mm past that origin, drove into the belt: the contact fight
        # spun wrist_2 ~25 turns (2026-09-27). +0.02 keeps the tips ~5 mm
        # above the belt while still gripping ~40 mm of the cube.
        self.declare_parameter('grasp_z_offset', 0.02)
        self.declare_parameter('approach_height', 0.15)
        self.declare_parameter('lift_height', 0.15)

        # --- where the cube goes -------------------------------------------
        # Above the bin's near-inner region, NOT its centre. From
        # scene_measure.txt the bin (/World/Group/Sorting_Bin_1) centres at
        # (+0.899, -0.289, -0.352) in base_link -- 1.008 m out, beyond the
        # UR5e's 0.850 m reach. Its near rim is reachable; its middle is not.
        # These numbers were derived by hand from that file and are NOT yet
        # verified against a successful plan.
        self.declare_parameter('place_xyz', [0.68, -0.22, -0.12])
        self.declare_parameter('place_approach_height', 0.15)

        # --- gripper --------------------------------------------------------
        # Driven straight over /joint_command: the gripper is geometry-only in
        # the URDF and deliberately not a MoveIt planning group (see ISSUES.md).
        self.declare_parameter('gripper_joint', 'finger_joint')
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('gripper_closed', 0.7)
        self.declare_parameter('gripper_settle_sec', 1.0)
        self.declare_parameter('command_topic', '/joint_command')

        # --- planning -------------------------------------------------------
        # Pilz, not OMPL: deterministic paths. Each move picks PTP (joint-space
        # sweep, for the long moves) or LIN (straight TCP line, for descend and
        # lift, which also can't flip to a different IK branch mid-move).
        self.declare_parameter('pipeline_id', 'pilz_industrial_motion_planner')
        # The sequence asks for a 'PTP' (sweep) or 'LIN' (straight line) move;
        # these name what the active pipeline calls each. cuMotion has one
        # planner for both: pass 'cuMotion' to each.
        self.declare_parameter('ptp_planner_id', 'PTP')
        self.declare_parameter('lin_planner_id', 'LIN')
        # Solve IK via move_group's /compute_ik -- seeded from the CURRENT arm
        # state -- and send the result as a joint goal. cuMotion solves pose
        # goals with its own many-seed IK and may pick a far branch: on
        # 2026-10-10 "above bin" flipped the elbow (+1.68 -> -0.34 rad) and
        # swung shoulder_lift 118 deg. Off for Pilz, whose IK already seeds
        # from the current state.
        self.declare_parameter('ik_then_joint_goal', False)
        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('acceleration_scaling', 0.3)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 10)
        self.declare_parameter('position_tolerance', 0.005)
        self.declare_parameter('orientation_tolerance', 0.05)

        # ISSUES.md's standing rule: check base->target distance BEFORE blaming
        # the planner. A pose outside this radius is rejected here, where the
        # message says so, rather than surfacing as an opaque planning failure.
        self.declare_parameter('max_reach', 0.850)

        self.declare_parameter('cube_pose_topic', '/cube_pose')
        self.declare_parameter('cube_pose_max_age_sec', 5.0)
        self.declare_parameter('auto_start', True)

        g = self.get_parameter
        self.group_name = g('group_name').value
        self.tcp_link = g('tcp_link').value
        self.tcp_offset_z = float(g('tcp_offset_z').value)
        self.base_frame = g('base_frame').value
        self.grasp_z_offset = float(g('grasp_z_offset').value)
        self.approach_height = float(g('approach_height').value)
        self.lift_height = float(g('lift_height').value)
        self.place_xyz = [float(v) for v in g('place_xyz').value]
        self.place_approach_height = float(g('place_approach_height').value)
        self.gripper_joint = g('gripper_joint').value
        self.gripper_open = float(g('gripper_open').value)
        self.gripper_closed = float(g('gripper_closed').value)
        self.gripper_settle = float(g('gripper_settle_sec').value)
        self.pipeline_id = g('pipeline_id').value
        self.planner_ids = {'PTP': g('ptp_planner_id').value,
                            'LIN': g('lin_planner_id').value}
        self.ik_then_joint_goal = bool(g('ik_then_joint_goal').value)
        self.vel_scale = float(g('velocity_scaling').value)
        self.acc_scale = float(g('acceleration_scaling').value)
        self.planning_time = float(g('planning_time').value)
        self.planning_attempts = int(g('planning_attempts').value)
        self.pos_tol = float(g('position_tolerance').value)
        self.ori_tol = float(g('orientation_tolerance').value)
        self.max_reach = float(g('max_reach').value)
        self.max_pose_age = float(g('cube_pose_max_age_sec').value)

        self._cube_pose = None
        self._cube_lock = threading.Lock()
        self._busy = threading.Lock()

        cb = ReentrantCallbackGroup()
        self._move = ActionClient(self, MoveGroup, '/move_action',
                                  callback_group=cb)
        self._ik = self.create_client(GetPositionIK, '/compute_ik',
                                      callback_group=cb)
        self._gripper_pub = self.create_publisher(
            JointState, g('command_topic').value, 10)
        self.create_subscription(
            PoseStamped, g('cube_pose_topic').value, self._on_cube, 10,
            callback_group=cb)
        self.create_service(Trigger, '~/pick_once', self._on_trigger,
                            callback_group=cb)

        self.get_logger().info(
            'pick_orchestrator up. PLANNING SCENE IS EMPTY -- the arm may '
            'plan through the conveyor, table and bin. Sim only.')

        if g('auto_start').value:
            threading.Thread(target=self._auto_start, daemon=True).start()

    # -- inputs --------------------------------------------------------------
    def _on_cube(self, msg):
        with self._cube_lock:
            self._cube_pose = msg

    def _on_trigger(self, _request, response):
        """Start a pick and return immediately.

        The sequence must NOT run inside this callback: ActionClient.send_goal()
        blocks until the action completes and deadlocks if called from one. So
        the service only starts a worker and reports whether it started -- watch
        the log for the outcome.
        """
        if self._busy.locked():
            response.success = False
            response.message = 'a pick is already running'
            return response
        threading.Thread(target=self.run_sequence, daemon=True).start()
        response.success = True
        response.message = 'pick started -- see the log for the outcome'
        return response

    def _auto_start(self):
        # Give move_group, the bridge and perception a moment to appear before
        # the first plan, rather than failing on a race at startup.
        self._sleep_clock(2.0)
        self.run_sequence()

    # -- time ----------------------------------------------------------------
    def _sleep_clock(self, seconds, wall_timeout=None):
        """Wait `seconds` of NODE-clock time (sim time under use_sim_time).

        Same reasoning as trajectory_bridge._sleep_clock: /clock only advances
        while Isaac Sim plays, so this polls and is bounded in wall time to
        avoid hanging forever if the sim is paused.
        """
        t0, w0 = self.get_clock().now().nanoseconds * 1e-9, time.monotonic()
        limit = wall_timeout if wall_timeout is not None else max(10.0, seconds * 10)
        while rclpy.ok():
            if (self.get_clock().now().nanoseconds * 1e-9) - t0 >= seconds:
                return True
            if time.monotonic() - w0 > limit:
                self.get_logger().warn(
                    f'clock did not advance {seconds:.2f}s within {limit:.0f}s '
                    'of wall time -- is Isaac Sim paused?')
                return False
            time.sleep(0.02)
        return False

    # -- goal construction ---------------------------------------------------
    def _base_goal(self, planner_id):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = self.planner_ids.get(planner_id, planner_id)
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        # Plan from wherever the arm actually is.
        req.start_state.is_diff = True

        req.workspace_parameters.header.frame_id = self.base_frame
        req.workspace_parameters.min_corner.x = -1.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z = -1.5
        req.workspace_parameters.max_corner.x = 1.5
        req.workspace_parameters.max_corner.y = 1.5
        req.workspace_parameters.max_corner.z = 1.5

        # An EMPTY diff: leave the planning scene exactly as move_group has it.
        # is_diff must be set, or move_group reads this as "replace the scene
        # with nothing", which would also wipe the robot state.
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        return goal

    def _pose_constraints(self, xyz, quat):
        c = Constraints()
        c.name = 'pose_goal'

        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tcp_link
        pc.weight = 1.0
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [self.pos_tol]
        volume = BoundingVolume()
        volume.primitives = [region]
        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        volume.primitive_poses = [pose]
        pc.constraint_region = volume
        c.position_constraints = [pc]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.tcp_link
        oc.orientation.x, oc.orientation.y, oc.orientation.z, \
            oc.orientation.w = quat
        oc.absolute_x_axis_tolerance = self.ori_tol
        oc.absolute_y_axis_tolerance = self.ori_tol
        oc.absolute_z_axis_tolerance = self.ori_tol
        oc.weight = 1.0
        c.orientation_constraints = [oc]
        return c

    def _joint_constraints(self, positions):
        c = Constraints()
        c.name = 'joint_goal'
        c.joint_constraints = [
            JointConstraint(joint_name=name, position=float(value),
                            tolerance_above=0.01, tolerance_below=0.01,
                            weight=1.0)
            for name, value in zip(ARM_JOINTS, positions)]
        return c

    # -- motion --------------------------------------------------------------
    def _send(self, goal, label):
        if not self._move.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                '/move_action not available -- is moveit.launch.py running?')
            return False

        # ActionClient.send_goal() returns the RESULT, not a goal handle --
        # "Unlike send_goal_async, this method returns the final result of the
        # action (not a goal handle)". It blocks until the action finishes, so
        # it must never be called from a callback; run_sequence() is driven from
        # a worker thread for exactly that reason.
        try:
            response = self._move.send_goal(goal)
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().error(f'{label}: send_goal raised: {exc}')
            return False

        if response is None:
            self.get_logger().error(f'{label}: move_group returned no result')
            return False

        code = response.result.error_code.val
        if code != MOVEIT_SUCCESS:
            self.get_logger().error(
                f'{label}: failed, MoveItErrorCode {code}. If this is a '
                'planning failure, re-test with a joint-space goal (home) to '
                'tell "bad goal" from "broken stack" -- see ISSUES.md.')
            return False

        self.get_logger().info(f'{label}: done')
        return True

    def _check_reach(self, xyz, label):
        d = math.sqrt(sum(v * v for v in xyz))
        if d > self.max_reach:
            self.get_logger().error(
                f'{label}: target ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f}) '
                f'is {d:.3f} m from base_link, beyond the {self.max_reach:.3f} m '
                'reach. Not attempting -- move the target or the robot.')
            return False
        self.get_logger().info(
            f'{label}: target ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f}), '
            f'{d:.3f} m from base')
        return True

    def move_to_pose(self, xyz, yaw, label, planner_id='PTP'):
        if not self._check_reach(xyz, label):
            return False
        # grasp_quaternion always points the tool straight down, so the
        # fingertips sitting tcp_offset_z past tcp_link means tcp_link must
        # be that much HIGHER in base_link.
        tcp_xyz = (xyz[0], xyz[1], xyz[2] + self.tcp_offset_z)
        quat = grasp_quaternion(yaw)
        goal = self._base_goal(planner_id)
        if self.ik_then_joint_goal:
            joints = self._solve_ik(tcp_xyz, quat, label)
            if joints is None:
                return False
            goal.request.goal_constraints = [self._joint_constraints(joints)]
        else:
            goal.request.goal_constraints = [
                self._pose_constraints(tcp_xyz, quat)]
        return self._send(goal, f'{label} [{planner_id}]')

    def _solve_ik(self, xyz, quat, label):
        """Joint angles for tcp_link at (xyz, quat), nearest the current state.

        move_group's /compute_ik starts from its current robot state and
        applies ik_request.robot_state as a diff -- an empty diff therefore
        seeds the solver from where the arm is now. Blocking call: only ever
        run from the sequence's worker thread, like _send().
        """
        if not self._ik.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'{label}: /compute_ik not available')
            return None
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group_name
        r.ik_link_name = self.tcp_link
        r.robot_state.is_diff = True
        r.avoid_collisions = True
        r.timeout.sec = 1
        r.pose_stamped.header.frame_id = self.base_frame
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x, p.orientation.y, p.orientation.z, \
            p.orientation.w = quat
        res = self._ik.call(req)
        if res is None or res.error_code.val != MOVEIT_SUCCESS:
            code = None if res is None else res.error_code.val
            self.get_logger().error(
                f'{label}: no IK solution near the current state '
                f'(MoveItErrorCode {code}). Not falling back to a pose goal, '
                'which could flip to a far branch.')
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM_JOINTS]

    def move_home(self):
        goal = self._base_goal('PTP')
        goal.request.goal_constraints = [self._joint_constraints(HOME_JOINTS)]
        return self._send(goal, 'home')

    def set_gripper(self, position, label):
        """Command finger_joint directly and wait for the fingers to settle.

        No feedback loop: Isaac publishes the gripper joints under different
        names than the URDF uses, so there is nothing to verify against here.
        The settle wait is open-loop by necessity.
        """
        if self._gripper_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                'no subscriber on the command topic -- is Isaac Sim playing? '
                'The gripper command will be dropped.')
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.gripper_joint]
        msg.position = [float(position)]
        self._gripper_pub.publish(msg)
        self.get_logger().info(f'{label}: {self.gripper_joint} -> {position:.3f}')
        self._sleep_clock(self.gripper_settle)
        return True

    # -- perception ----------------------------------------------------------
    def latch_cube_pose(self):
        """One fresh cube pose, then stop listening. See module docstring."""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and rclpy.ok():
            with self._cube_lock:
                pose = self._cube_pose
            if pose is not None:
                stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
                age = (self.get_clock().now().nanoseconds * 1e-9) - stamp
                # Negative age = stamped in the future = the sim clock was reset
                # (Isaac Stop -> Play restarts /clock at 0). That pose predates
                # the reset, so the cube may since have moved: reject it.
                if 0.0 <= age <= self.max_pose_age:
                    p = pose.pose.position
                    q = pose.pose.orientation
                    yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
                    self.get_logger().info(
                        f'cube latched at ({p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f}) '
                        f'yaw {math.degrees(yaw):+.1f} deg, age {age:.2f}s')
                    return (p.x, p.y, p.z), yaw
                self.get_logger().warn(
                    f'cube pose is {age:.1f}s stale (limit {self.max_pose_age:.1f}s)',
                    throttle_duration_sec=2.0)
            time.sleep(0.1)
        self.get_logger().error(
            'no fresh cube pose. Is cube_pose_publisher running and the tag '
            'in view? Note both this node and perception need use_sim_time.')
        return None, None

    # -- the sequence --------------------------------------------------------
    def run_sequence(self):
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('a pick is already running')
            return False
        try:
            return self._run_sequence()
        finally:
            self._busy.release()

    def _run_sequence(self):
        self.get_logger().info('--- pick sequence start ---')

        if not self.move_home():
            return False
        if not self.set_gripper(self.gripper_open, 'open gripper'):
            return False

        cube, yaw = self.latch_cube_pose()
        if cube is None:
            return False

        grasp = (cube[0], cube[1], cube[2] + self.grasp_z_offset)
        above = (grasp[0], grasp[1], grasp[2] + self.approach_height)
        lift = (grasp[0], grasp[1], grasp[2] + self.lift_height)
        place = tuple(self.place_xyz)
        above_place = (place[0], place[1], place[2] + self.place_approach_height)

        steps = [
            ('approach cube', lambda: self.move_to_pose(above, yaw, 'approach cube')),
            ('descend to grasp', lambda: self.move_to_pose(grasp, yaw, 'descend to grasp', 'LIN')),
            ('close gripper', lambda: self.set_gripper(self.gripper_closed, 'close gripper')),
            ('lift', lambda: self.move_to_pose(lift, yaw, 'lift', 'LIN')),
            ('above bin', lambda: self.move_to_pose(above_place, 0.0, 'above bin')),
            ('lower into bin', lambda: self.move_to_pose(place, 0.0, 'lower into bin', 'LIN')),
            ('release', lambda: self.set_gripper(self.gripper_open, 'release')),
            ('retreat', lambda: self.move_to_pose(above_place, 0.0, 'retreat', 'LIN')),
            ('home', self.move_home),
        ]

        for name, step in steps:
            if not step():
                self.get_logger().error(
                    f'--- pick sequence ABORTED at "{name}" ---')
                return False

        self.get_logger().info('--- pick sequence complete ---')
        return True


def main():
    rclpy.init()
    node = PickOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
