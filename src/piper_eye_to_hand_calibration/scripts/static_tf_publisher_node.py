#!/usr/bin/env python3

import os

import numpy as np
import rospy
import tf2_ros
import yaml

from piper_eye_to_hand_calibration.ros_support import (
    matrix_to_transform_stamped,
    normalize_frame,
    normalize_serial,
    transform_stamped_to_matrix,
    wait_for_camera_identity,
)
from piper_eye_to_hand_calibration.transforms import (
    transform_distance,
    validate_transform,
)


class ResultPublisher:
    def __init__(self):
        result_files = rospy.get_param("~result_files")
        expected_cameras = rospy.get_param("~expected_cameras")
        expected_serials = rospy.get_param("~expected_serials")
        if not isinstance(result_files, list):
            result_files = [result_files]
        if not isinstance(expected_cameras, list):
            expected_cameras = [expected_cameras]
        if not isinstance(expected_serials, list):
            expected_serials = [expected_serials]
        if not (
            len(result_files) == len(expected_cameras) == len(expected_serials)
            and len(result_files) in (1, 2)
        ):
            raise ValueError("result_files, expected_cameras and expected_serials must have length 1 or 2")

        normalized_serials = [normalize_serial(item) for item in expected_serials]
        if any(not item for item in normalized_serials):
            raise ValueError("expected serial numbers must not be empty")
        if len(set(normalized_serials)) != len(normalized_serials):
            raise ValueError("duplicate camera serial numbers are not allowed")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        pending = []
        for path, camera, serial in zip(result_files, expected_cameras, normalized_serials):
            pending.append(self._validate_result(path, camera, serial))

        messages = [
            matrix_to_transform_stamped(matrix, parent, child)
            for parent, child, matrix in pending
        ]
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster.sendTransform(messages)
        for message in messages:
            rospy.loginfo(
                "Published static transform %s -> %s",
                message.header.frame_id,
                message.child_frame_id,
            )

    @staticmethod
    def _matrix(record):
        return validate_transform(np.asarray(record["matrix_4x4"], dtype=np.float64))

    def _validate_result(self, path, expected_camera, expected_serial):
        expected_camera = str(expected_camera).strip().strip("/")
        if expected_camera not in ("rs1", "rs3"):
            raise ValueError("expected camera must be rs1 or rs3")
        path = os.path.abspath(os.path.expanduser(str(path)))
        with open(path, "r", encoding="utf-8") as source:
            result = yaml.safe_load(source)
        if not isinstance(result, dict) or result.get("schema_version") != 1:
            raise ValueError("unsupported result schema: {}".format(path))
        if result.get("status") != "PASS" or not result.get("quality", {}).get("passed"):
            raise ValueError("refusing to publish a non-PASS result: {}".format(path))

        camera = result.get("camera", {})
        actual_camera = str(camera.get("name", ""))
        actual_serial = normalize_serial(camera.get("serial", ""))
        if actual_camera != expected_camera:
            raise ValueError(
                "result camera mismatch: expected {}, got {}".format(expected_camera, actual_camera)
            )
        if actual_serial != expected_serial:
            raise ValueError(
                "result serial mismatch for {}: expected {}, got {}".format(
                    expected_camera, expected_serial, actual_serial
                )
            )
        intrinsics = result.get("intrinsics", {})
        if intrinsics.get("source") != "calibrated_file":
            raise ValueError("result does not record calibrated file intrinsics: {}".format(path))
        if str(intrinsics.get("camera", "")) != expected_camera:
            raise ValueError("result intrinsics camera mismatch: {}".format(path))
        if normalize_serial(intrinsics.get("camera_serial_number", "")) != expected_serial:
            raise ValueError("result intrinsics serial mismatch: {}".format(path))
        wait_for_camera_identity(expected_camera, expected_serial, timeout_sec=10.0)

        publication = result["tf_publication"]
        base_to_link_record = publication["base_to_camera_link"]
        saved_internal_record = publication["camera_link_to_optical"]
        base_to_optical_record = result["calibration"]["base_to_camera_optical"]
        base_to_link = self._matrix(base_to_link_record)
        saved_internal = self._matrix(saved_internal_record)
        base_to_optical = self._matrix(base_to_optical_record)

        parent = normalize_frame(base_to_link_record["parent_frame"])
        if parent != "arm_base" or normalize_frame(base_to_optical_record["parent_frame"]) != parent:
            raise ValueError("result base frame must be arm_base")
        link_frame = "{}_link".format(expected_camera)
        optical_frame = "{}_color_optical_frame".format(expected_camera)
        if normalize_frame(base_to_link_record["child_frame"]) != link_frame:
            raise ValueError("unexpected link frame in {}".format(path))
        if normalize_frame(saved_internal_record["parent_frame"]) != link_frame:
            raise ValueError("unexpected internal TF parent in {}".format(path))
        if normalize_frame(saved_internal_record["child_frame"]) != optical_frame:
            raise ValueError("unexpected internal TF child in {}".format(path))
        if normalize_frame(base_to_optical_record["child_frame"]) != optical_frame:
            raise ValueError("unexpected optical frame in {}".format(path))

        current_internal_message = self.tf_buffer.lookup_transform(
            link_frame,
            optical_frame,
            rospy.Time(0),
            rospy.Duration(2.0),
        )
        current_internal = transform_stamped_to_matrix(current_internal_message)
        translation_error, rotation_error = transform_distance(saved_internal, current_internal)
        if translation_error > 1e-4 or rotation_error > 0.05:
            raise ValueError(
                "RealSense internal TF changed for {}: {:.6f} m / {:.3f} deg".format(
                    expected_camera, translation_error, rotation_error
                )
            )

        composed = base_to_link @ current_internal
        translation_error, rotation_error = transform_distance(composed, base_to_optical)
        if translation_error > 1e-6 or rotation_error > 1e-4:
            raise ValueError(
                "stored TF composition is inconsistent for {}".format(expected_camera)
            )
        rospy.loginfo("Validated %s result %s for serial %s", expected_camera, path, expected_serial)
        return parent, link_frame, base_to_link


def main():
    rospy.init_node("eye_to_hand_static_tf_publisher")
    try:
        ResultPublisher()
        rospy.spin()
    except Exception as exception:
        rospy.logfatal("Static TF publisher refused to start: %s", exception)
        raise


if __name__ == "__main__":
    main()
