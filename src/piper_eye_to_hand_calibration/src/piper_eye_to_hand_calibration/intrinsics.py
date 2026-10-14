"""Load and validate the calibrated color-camera intrinsic profiles."""

import hashlib
import os

import numpy as np
import yaml


def _matrix(data, rows, cols, name):
    if not isinstance(data, dict):
        raise ValueError("{} must be a matrix record".format(name))
    if int(data.get("rows", -1)) != rows or int(data.get("cols", -1)) != cols:
        raise ValueError("{} must be {}x{}".format(name, rows, cols))
    values = np.asarray(data.get("data", []), dtype=np.float64)
    if values.size != rows * cols or not np.all(np.isfinite(values)):
        raise ValueError("{} contains invalid values".format(name))
    return values.reshape(rows, cols)


def load_intrinsics(path, expected_camera=None, expected_serial=None):
    path = os.path.abspath(os.path.expanduser(str(path)))
    with open(path, "rb") as source:
        contents = source.read()
    profile = yaml.safe_load(contents)
    result = validate_profile(profile, expected_camera, expected_serial)
    result.update(path=path, sha256=hashlib.sha256(contents).hexdigest())
    return result


def validate_profile(profile, expected_camera=None, expected_serial=None):
    if not isinstance(profile, dict):
        raise ValueError("intrinsics file must contain a mapping")

    camera = str(profile.get("camera", ""))
    serial = str(profile.get("camera_serial_number", "")).lstrip("_")
    if expected_camera is not None and camera != str(expected_camera):
        raise ValueError(
            "intrinsics camera mismatch: expected {}, got {}".format(expected_camera, camera)
        )
    if expected_serial is not None and serial != str(expected_serial).lstrip("_"):
        raise ValueError(
            "intrinsics serial mismatch: expected {}, got {}".format(expected_serial, serial)
        )

    width = int(profile.get("image_width", 0))
    height = int(profile.get("image_height", 0))
    if width <= 0 or height <= 0:
        raise ValueError("intrinsics image dimensions must be positive")
    if profile.get("distortion_model") != "plumb_bob":
        raise ValueError("only plumb_bob intrinsics are supported")

    camera_matrix = _matrix(profile.get("camera_matrix"), 3, 3, "camera_matrix")
    distortion = _matrix(
        profile.get("distortion_coefficients"), 1, 5, "distortion_coefficients"
    ).reshape(5)
    rectification = _matrix(
        profile.get("rectification_matrix"), 3, 3, "rectification_matrix"
    )
    projection = _matrix(profile.get("projection_matrix"), 3, 4, "projection_matrix")
    if (camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0
            or not np.allclose(camera_matrix[2], [0, 0, 1])
            or not np.allclose(camera_matrix[1, 0], 0)):
        raise ValueError("invalid pinhole camera matrix")
    if (not np.allclose(rectification.T @ rectification, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rectification), 1, atol=1e-6)
            or projection[0, 0] <= 0 or projection[1, 1] <= 0
            or not np.allclose(projection[2], [0, 0, 1, 0])):
        raise ValueError("invalid rectification or projection matrix")

    return {
        "camera": camera,
        "serial": serial,
        "camera_model": str(profile.get("camera_model", "")),
        "calibration_time": str(profile.get("calibration_time", "")),
        "calibration_target": str(profile.get("calibration_target", "")),
        "source_record": str(profile.get("source_record", "")),
        "width": width,
        "height": height,
        "distortion_model": "plumb_bob",
        "camera_matrix": camera_matrix,
        "distortion": distortion,
        "rectification": rectification,
        "projection": projection,
    }
