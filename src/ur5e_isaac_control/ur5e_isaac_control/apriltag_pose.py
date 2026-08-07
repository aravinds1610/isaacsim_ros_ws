#!/usr/bin/env python3
"""AprilTag detection + 6-DOF pose estimation using OpenCV (zero extra deps).

OpenCV 4.12's ``cv2.aruco`` ships the AprilTag families, so no apriltag_ros /
pupil-apriltags needed. ``estimatePoseSingleMarkers`` was removed in 4.7+, so we
recover pose with ``cv2.solvePnP`` on the four detected tag corners against the
tag's known metric corner geometry.

Pose convention (matches the old estimatePoseSingleMarkers): the returned 4x4 is
``T_cam_tag`` — the tag frame expressed in the camera optical frame (OpenCV
optical frame: X right, Y down, Z forward/out of the lens). The tag frame sits
at the tag centre with X right, Y up, Z out of the tag face toward the viewer.
"""
import cv2
import numpy as np

# 36h11 is the standard, most robust AprilTag family.
DEFAULT_FAMILY = 'DICT_APRILTAG_36h11'


class AprilTagDetector:
    def __init__(self, tag_size_m, family=DEFAULT_FAMILY):
        """tag_size_m: physical side length of the black tag square, in metres."""
        self.tag_size = float(tag_size_m)
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, family))
        params = cv2.aruco.DetectorParameters()
        # Sub-pixel corner refinement -> markedly better pose accuracy.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        # Tag corners in the tag frame, ordered to match aruco's output:
        # top-left, top-right, bottom-right, bottom-left. X right, Y up, Z out.
        h = self.tag_size / 2.0
        self.obj_points = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

    def detect(self, image, camera_matrix, dist_coeffs=None):
        """Return list of dicts: {id, corners(4x2), T_cam_tag(4x4), rvec, tvec}.

        image: BGR or grayscale. camera_matrix: 3x3 intrinsics (K).
        """
        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        if dist_coeffs is None:
            dist_coeffs = np.zeros(5, dtype=np.float32)

        corners, ids, _ = self.detector.detectMarkers(gray)
        results = []
        if ids is None:
            return results

        for tag_corners, tag_id in zip(corners, ids.flatten()):
            img_points = tag_corners.reshape(4, 2).astype(np.float32)
            pose = self.pose_from_corners(img_points, camera_matrix, dist_coeffs)
            if pose is None:
                continue
            T, rvec, tvec = pose
            results.append({
                'id': int(tag_id),
                'corners': img_points,
                'T_cam_tag': T,
                'rvec': rvec,
                'tvec': tvec,
            })
        return results

    def pose_from_corners(self, img_points, camera_matrix, dist_coeffs=None):
        """4x2 tag corners (TL,TR,BR,BL) -> (T_cam_tag 4x4, rvec, tvec) or None.

        Decoupled from detection so the pose math can be unit-tested directly.
        """
        if dist_coeffs is None:
            dist_coeffs = np.zeros(5, dtype=np.float32)
        img_points = np.asarray(img_points, dtype=np.float32).reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_points, img_points, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)  # optimal for planar squares
        if not ok:
            return None
        T = np.eye(4)
        T[:3, :3], _ = cv2.Rodrigues(rvec)
        T[:3, 3] = tvec.flatten()
        return T, rvec, tvec.flatten()


def camera_matrix_from_info(width, height, fx, fy, cx, cy):
    """Build a 3x3 K. (cx, cy default to image centre if None.)"""
    cx = width / 2.0 if cx is None else cx
    cy = height / 2.0 if cy is None else cy
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


def draw_detections(image, detections, camera_matrix, axis_len=0.03):
    """Overlay tag outlines + coordinate axes for debugging. Returns a copy."""
    out = image.copy() if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for d in detections:
        cv2.polylines(out, [d['corners'].astype(int)], True, (0, 255, 0), 2)
        cv2.drawFrameAxes(out, camera_matrix, np.zeros(5),
                          d['rvec'], d['tvec'], axis_len)
        c = d['corners'].mean(axis=0).astype(int)
        cv2.putText(out, f"id{d['id']}", tuple(c),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return out
