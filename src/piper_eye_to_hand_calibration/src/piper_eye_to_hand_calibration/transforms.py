"""Pure transform and eye-to-hand calibration helpers."""

import math

import cv2
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix


def normalize_quaternion(quaternion):
    quaternion = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    return quaternion / norm


def transform_from_xyz_quaternion(translation, quaternion):
    matrix = quaternion_matrix(normalize_quaternion(quaternion))
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("transform contains non-finite values")
    return matrix


def transform_to_xyz_quaternion(matrix):
    matrix = validate_transform(matrix)
    quaternion = normalize_quaternion(quaternion_from_matrix(matrix))
    return matrix[:3, 3].copy(), quaternion


def validate_transform(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("transform must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("invalid homogeneous transform bottom row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ValueError("rotation matrix is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise ValueError("rotation matrix determinant is not one")
    return matrix


def rotation_distance_deg(first, second):
    relative = validate_transform(first)[:3, :3].T @ validate_transform(second)[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def transform_distance(first, second):
    first = validate_transform(first)
    second = validate_transform(second)
    translation = float(np.linalg.norm(first[:3, 3] - second[:3, 3]))
    return translation, rotation_distance_deg(first, second)


def _mean_quaternion(quaternions):
    accumulator = np.zeros((4, 4), dtype=np.float64)
    for quaternion in quaternions:
        q = normalize_quaternion(quaternion)
        accumulator += np.outer(q, q)
    _, eigenvectors = np.linalg.eigh(accumulator)
    result = normalize_quaternion(eigenvectors[:, -1])
    if result[3] < 0.0:
        result = -result
    return result


def mean_transform(transforms):
    if not transforms:
        raise ValueError("at least one transform is required")
    transforms = [validate_transform(item) for item in transforms]
    translation = np.mean([item[:3, 3] for item in transforms], axis=0)
    quaternion = _mean_quaternion([quaternion_from_matrix(item) for item in transforms])
    return transform_from_xyz_quaternion(translation, quaternion)


def solve_eye_to_hand(base_to_tool, camera_to_target):
    """Return base_T_camera from paired base_T_tool and camera_T_target samples."""
    if len(base_to_tool) != len(camera_to_target):
        raise ValueError("robot and marker sample counts differ")
    if len(base_to_tool) < 3:
        raise ValueError("at least three samples are required by OpenCV")

    base_to_tool = [validate_transform(item) for item in base_to_tool]
    camera_to_target = [validate_transform(item) for item in camera_to_target]
    tool_to_base = [np.linalg.inv(item) for item in base_to_tool]

    rotation, translation = cv2.calibrateHandEye(
        [item[:3, :3] for item in tool_to_base],
        [item[:3, 3] for item in tool_to_base],
        [item[:3, :3] for item in camera_to_target],
        [item[:3, 3] for item in camera_to_target],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation).reshape(3)
    return validate_transform(result)


def consistency_metrics(base_to_tool, camera_to_target, base_to_camera):
    """Measure how constant tool_T_target is across all calibration samples."""
    tool_to_target = [
        np.linalg.inv(validate_transform(base_tool))
        @ validate_transform(base_to_camera)
        @ validate_transform(camera_target)
        for base_tool, camera_target in zip(base_to_tool, camera_to_target)
    ]
    center = mean_transform(tool_to_target)
    translation_errors = []
    rotation_errors = []
    for item in tool_to_target:
        translation, rotation = transform_distance(center, item)
        translation_errors.append(translation)
        rotation_errors.append(rotation)

    return {
        "tool_to_target_mean": center,
        "translation_rms_m": float(np.sqrt(np.mean(np.square(translation_errors)))),
        "translation_max_m": float(np.max(translation_errors)),
        "rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotation_errors)))),
        "rotation_max_deg": float(np.max(rotation_errors)),
    }


def matrix_to_list(matrix):
    return validate_transform(matrix).tolist()
