#!/usr/bin/env python3

import unittest

import cv2
import numpy as np

from piper_eye_to_hand_calibration.aruco_support import (
    create_detector,
    detect_markers,
    estimate_marker_pose,
    marker_object_points,
)


class ArucoSupportTest(unittest.TestCase):
    def test_detects_configured_dictionary_and_id(self):
        dictionary, parameters, detector = create_detector()
        marker = np.zeros((200, 200), dtype=np.uint8)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(dictionary, 18, 200)
        else:
            cv2.aruco.drawMarker(dictionary, 18, 200, marker, 1)
        canvas = np.full((400, 400), 255, dtype=np.uint8)
        canvas[100:300, 100:300] = marker
        corners, identifiers, _ = detect_markers(canvas, dictionary, parameters, detector)
        self.assertIsNotNone(identifiers)
        self.assertIn(18, np.asarray(identifiers).reshape(-1).tolist())
        self.assertEqual(len(corners), 1)

    def test_pose_scale_from_projected_100mm_marker(self):
        camera_matrix = np.array(
            [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        distortion = np.zeros(5, dtype=np.float64)
        rotation_vector = np.array([0.08, -0.12, 0.04], dtype=np.float64)
        translation = np.array([0.02, -0.01, 0.60], dtype=np.float64)
        corners, _ = cv2.projectPoints(
            marker_object_points(0.100),
            rotation_vector,
            translation,
            camera_matrix,
            distortion,
        )
        transform, error, _ = estimate_marker_pose(
            corners, camera_matrix, distortion, 0.100
        )
        np.testing.assert_allclose(transform[:3, 3], translation, atol=1e-8)
        self.assertLess(error, 1e-8)


if __name__ == "__main__":
    unittest.main()
