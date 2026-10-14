#!/usr/bin/env python3

import unittest

import cv2
import numpy as np

from piper_eye_to_hand_calibration.transforms import (
    consistency_metrics,
    rotation_distance_deg,
    solve_eye_to_hand,
    transform_distance,
    validate_transform,
)


def make_transform(rotation_vector, translation):
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3], _ = cv2.Rodrigues(np.asarray(rotation_vector, dtype=np.float64))
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


class CalibrationMathTest(unittest.TestCase):
    def setUp(self):
        self.base_to_camera = make_transform([0.25, -0.18, 0.31], [0.48, -0.22, 0.72])
        self.tool_to_target = make_transform([-0.12, 0.08, 0.16], [0.02, 0.01, 0.14])
        random = np.random.RandomState(7)
        self.base_to_tool = []
        self.camera_to_target = []
        for _ in range(24):
            robot_pose = make_transform(
                random.uniform(-1.0, 1.0, size=3),
                random.uniform([-0.25, -0.30, 0.20], [0.50, 0.35, 0.70]),
            )
            marker_pose = np.linalg.inv(self.base_to_camera) @ robot_pose @ self.tool_to_target
            self.base_to_tool.append(robot_pose)
            self.camera_to_target.append(marker_pose)

    def test_eye_to_hand_recovers_base_to_camera(self):
        result = solve_eye_to_hand(self.base_to_tool, self.camera_to_target)
        translation_error, rotation_error = transform_distance(result, self.base_to_camera)
        self.assertLess(translation_error, 1e-8)
        self.assertLess(rotation_error, 1e-6)

    def test_consistency_metrics_are_zero_for_exact_data(self):
        metrics = consistency_metrics(
            self.base_to_tool, self.camera_to_target, self.base_to_camera
        )
        self.assertLess(metrics["translation_rms_m"], 1e-10)
        self.assertLess(metrics["rotation_rms_deg"], 1e-6)

    def test_link_optical_composition(self):
        link_to_optical = make_transform([1.5, -0.1, 1.4], [0.02, 0.01, 0.03])
        base_to_link = self.base_to_camera @ np.linalg.inv(link_to_optical)
        composed = base_to_link @ link_to_optical
        np.testing.assert_allclose(composed, self.base_to_camera, atol=1e-10)

    def test_invalid_rotation_is_rejected(self):
        bad = np.eye(4)
        bad[0, 0] = 2.0
        with self.assertRaises(ValueError):
            validate_transform(bad)

    def test_rotation_distance(self):
        first = np.eye(4)
        second = make_transform([0.0, 0.0, np.pi / 2.0], [0.0, 0.0, 0.0])
        self.assertAlmostEqual(rotation_distance_deg(first, second), 90.0, places=6)


if __name__ == "__main__":
    unittest.main()
