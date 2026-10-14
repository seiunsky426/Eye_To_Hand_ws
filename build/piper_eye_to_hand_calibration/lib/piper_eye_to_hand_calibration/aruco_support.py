"""OpenCV-version-compatible ArUco detection and pose estimation."""

import cv2
import numpy as np


def create_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        parameters = cv2.aruco.DetectorParameters_create()
    else:
        parameters = cv2.aruco.DetectorParameters()
    if hasattr(parameters, "cornerRefinementMethod"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters) if hasattr(cv2.aruco, "ArucoDetector") else None
    return dictionary, parameters, detector


def detect_markers(image, dictionary, parameters, detector):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    if detector is not None:
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def marker_object_points(marker_size_m):
    half = float(marker_size_m) / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def estimate_marker_pose(corners, camera_matrix, distortion, marker_size_m):
    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    object_points = marker_object_points(marker_size_m)
    success, rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        image_points,
        np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
        np.asarray(distortion, dtype=np.float64),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not success:
        raise ValueError("solvePnP failed")

    translation_vector = np.asarray(translation_vector, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(translation_vector)) or translation_vector[2] <= 0.0:
        raise ValueError("marker pose has invalid depth")

    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion,
    )
    projected = np.asarray(projected).reshape(4, 2)
    reprojection_error = float(np.sqrt(np.mean(np.sum((projected - image_points) ** 2, axis=1))))
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = translation_vector
    return transform, reprojection_error, np.asarray(rotation_vector).reshape(3)
