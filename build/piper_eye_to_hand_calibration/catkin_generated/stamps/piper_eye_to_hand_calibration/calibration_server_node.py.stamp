#!/usr/bin/env python3

import datetime
import os
import threading
import time
from collections import deque

import message_filters
import numpy as np
import rospy
import tf2_ros
import yaml
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse

from piper_eye_to_hand_calibration.intrinsics import load_intrinsics
from piper_eye_to_hand_calibration.artifacts import sha256, timestamp, write_yaml
from piper_eye_to_hand_calibration.stereo import bind_handeye
from piper_eye_to_hand_calibration.ros_support import (
    normalize_frame,
    pose_to_matrix,
    transform_stamped_to_matrix,
    wait_for_camera_identity,
)
from piper_eye_to_hand_calibration.transforms import (
    consistency_metrics,
    matrix_to_list,
    solve_eye_to_hand,
    transform_distance,
    transform_to_xyz_quaternion,
)


class CalibrationServer:
    def __init__(self):
        self.camera = rospy.get_param("~camera")
        self.serial = wait_for_camera_identity(
            self.camera,
            rospy.get_param("~camera_serial"),
            rospy.get_param("~serial_check_timeout", 10.0),
        )
        self.base_frame = normalize_frame(rospy.get_param("~base_frame", "arm_base"))
        self.camera_frame = normalize_frame(
            rospy.get_param("~camera_frame", "{}_color_optical_frame".format(self.camera))
        )
        self.camera_link_frame = normalize_frame(
            rospy.get_param("~camera_link_frame", "{}_link".format(self.camera))
        )
        self.robot_pose_topic = rospy.get_param("~robot_pose_topic", "/end_pose")
        self.marker_pose_topic = rospy.get_param("~marker_pose_topic", "marker_pose")
        self.image_topic = rospy.get_param(
            "~image_topic", "/{}/color/image_raw".format(self.camera)
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/{}/color/camera_info".format(self.camera)
        )
        self.intrinsics = load_intrinsics(
            rospy.get_param("~intrinsics_file"),
            expected_camera=self.camera,
            expected_serial=self.serial,
        )
        self.stereo_path = str(Path(rospy.get_param("~stereo_result")).expanduser().resolve())
        self.stereo, self.stereo_binding = bind_handeye(self.stereo_path, self.intrinsics)
        if self.base_frame != "arm_base" or self.camera_frame != "rs1_color_optical_frame" or self.camera_link_frame != "rs1_link":
            raise ValueError("workflow requires arm_base, rs1_link and rs1_color_optical_frame")
        self.result_dir = os.path.expanduser(rospy.get_param("~result_dir"))

        self.min_samples = int(rospy.get_param("~min_samples", 15))
        self.sync_slop_sec = float(rospy.get_param("~sync_slop_sec", 0.05))
        self.max_pair_age_sec = float(rospy.get_param("~max_pair_age_sec", 0.5))
        self.stationary_window_sec = float(rospy.get_param("~stationary_window_sec", 0.3))
        self.stationary_translation_m = float(
            rospy.get_param("~stationary_translation_m", 0.002)
        )
        self.stationary_rotation_deg = float(
            rospy.get_param("~stationary_rotation_deg", 1.0)
        )
        self.duplicate_translation_m = float(
            rospy.get_param("~duplicate_translation_m", 0.010)
        )
        self.duplicate_rotation_deg = float(
            rospy.get_param("~duplicate_rotation_deg", 5.0)
        )
        self.translation_rms_limit_m = float(
            rospy.get_param("~translation_rms_limit_m", 0.010)
        )
        self.rotation_rms_limit_deg = float(
            rospy.get_param("~rotation_rms_limit_deg", 1.5)
        )
        self.marker_id = int(rospy.get_param("~marker_id", 18))
        self.marker_size_m = float(rospy.get_param("~marker_size_m", 0.100))

        if self.min_samples < 15:
            raise ValueError("min_samples must be at least fifteen")
        if self.marker_id != 18 or not np.isclose(self.marker_size_m, .100):
            raise ValueError("workflow requires ID 18, 100 mm black-square marker")

        self.lock = threading.Lock()
        self.latest_pair = None
        self.robot_history = deque()
        self.samples = []
        self.session_dir = Path(self.result_dir) / timestamp()
        self.session_dir.mkdir(parents=True, exist_ok=False)
        write_yaml(self.session_dir / "session.yaml", {"stereo_binding": self.stereo_binding,
                   "intrinsics_file": self.intrinsics["path"], "intrinsics_sha256": self.intrinsics["sha256"]})

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot_history_subscriber = rospy.Subscriber(
            self.robot_pose_topic,
            PoseStamped,
            self.robot_history_callback,
            queue_size=100,
            tcp_nodelay=True,
        )
        self.robot_pose_subscriber = message_filters.Subscriber(
            self.robot_pose_topic, PoseStamped
        )
        self.marker_pose_subscriber = message_filters.Subscriber(
            self.marker_pose_topic, PoseStamped
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.robot_pose_subscriber, self.marker_pose_subscriber],
            queue_size=50,
            slop=self.sync_slop_sec,
        )
        self.synchronizer.registerCallback(self.pair_callback)

        self.capture_service = rospy.Service("capture", Trigger, self.capture)
        self.delete_service = rospy.Service("delete_last", Trigger, self.delete_last)
        self.reset_service = rospy.Service("reset", Trigger, self.reset)
        self.solve_service = rospy.Service("solve", Trigger, self.solve)
        rospy.loginfo(
            "%s calibration server ready: serial=%s, minimum samples=%d",
            self.camera,
            self.serial,
            self.min_samples,
        )

    def _valid_robot_frame(self, frame):
        frame = normalize_frame(frame)
        return not frame or frame == self.base_frame

    def robot_history_callback(self, message):
        stamp = message.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0 or age > .5 or age < -.1:
            return
        if not self._valid_robot_frame(message.header.frame_id):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring /end_pose frame %s; expected empty or %s",
                message.header.frame_id,
                self.base_frame,
            )
            return
        try:
            matrix = pose_to_matrix(message.pose)
        except (TypeError, ValueError) as exception:
            rospy.logwarn_throttle(2.0, "Invalid Piper pose: %s", exception)
            return
        now = time.monotonic()
        with self.lock:
            self.robot_history.append((now, matrix))
            oldest = now - self.stationary_window_sec - 0.5
            while self.robot_history and self.robot_history[0][0] < oldest:
                self.robot_history.popleft()

    def pair_callback(self, robot_message, marker_message):
        if not self._valid_robot_frame(robot_message.header.frame_id):
            return
        if normalize_frame(marker_message.header.frame_id) != self.camera_frame:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring marker frame %s; expected %s",
                marker_message.header.frame_id,
                self.camera_frame,
            )
            return
        robot_stamp = robot_message.header.stamp.to_sec()
        marker_stamp = marker_message.header.stamp.to_sec()
        if robot_stamp <= 0.0 or marker_stamp <= 0.0:
            rospy.logwarn_throttle(2.0, "Ignoring synchronized pair with a zero timestamp")
            return
        if abs(robot_stamp - marker_stamp) > self.sync_slop_sec:
            return
        now_ros = rospy.Time.now().to_sec()
        if any(now_ros - stamp > .5 or stamp - now_ros > .1 for stamp in (robot_stamp, marker_stamp)):
            return
        try:
            robot_matrix = pose_to_matrix(robot_message.pose)
            marker_matrix = pose_to_matrix(marker_message.pose)
        except (TypeError, ValueError) as exception:
            rospy.logwarn_throttle(2.0, "Invalid synchronized pose pair: %s", exception)
            return
        with self.lock:
            self.latest_pair = {
                "received_monotonic": time.monotonic(),
                "robot_stamp": robot_stamp,
                "marker_stamp": marker_stamp,
                "base_to_tool": robot_matrix,
                "camera_to_target": marker_matrix,
            }

    def _stationary_locked(self):
        if not self.robot_history:
            return False, "insufficient Piper history for stationary check: 0 samples"
        latest_feedback_time = self.robot_history[-1][0]
        recent = [
            (timestamp, matrix)
            for timestamp, matrix in self.robot_history
            if timestamp >= latest_feedback_time - self.stationary_window_sec
        ]
        history_span = recent[-1][0] - recent[0][0] if len(recent) >= 2 else 0.0
        if len(recent) < 2 or history_span < self.stationary_window_sec * 0.8:
            return False, (
                "insufficient Piper history for stationary check: "
                "{} samples over {:.3f} s"
            ).format(len(recent), history_span)
        reference = recent[-1][1]
        max_translation = 0.0
        max_rotation = 0.0
        for _, matrix in recent:
            translation, rotation = transform_distance(reference, matrix)
            max_translation = max(max_translation, translation)
            max_rotation = max(max_rotation, rotation)
        if (
            max_translation > self.stationary_translation_m
            or max_rotation > self.stationary_rotation_deg
        ):
            return False, "Piper is moving: {:.4f} m / {:.2f} deg".format(
                max_translation, max_rotation
            )
        return True, "stationary"

    def capture(self, _request):
        try:
            self._check_binding()
        except (OSError, ValueError) as exc:
            return TriggerResponse(False, str(exc))
        now = time.monotonic()
        with self.lock:
            if self.latest_pair is None:
                return TriggerResponse(False, "no synchronized Piper/marker observation")
            age = now - self.latest_pair["received_monotonic"]
            if age > self.max_pair_age_sec:
                return TriggerResponse(False, "latest observation is stale ({:.2f} s)".format(age))
            if any(rospy.Time.now().to_sec() - self.latest_pair[key] > self.max_pair_age_sec
                   for key in ("robot_stamp", "marker_stamp")):
                return TriggerResponse(False, "latest source timestamps are stale")
            stationary, reason = self._stationary_locked()
            if not stationary:
                return TriggerResponse(False, reason)

            candidate = self.latest_pair
            for existing in self.samples:
                translation, rotation = transform_distance(
                    existing["base_to_tool"], candidate["base_to_tool"]
                )
                if (
                    translation < self.duplicate_translation_m
                    and rotation < self.duplicate_rotation_deg
                ):
                    return TriggerResponse(
                        False,
                        "near-duplicate pose: {:.4f} m / {:.2f} deg".format(
                            translation, rotation
                        ),
                    )
            self.samples.append(
                {
                    "robot_stamp": candidate["robot_stamp"],
                    "marker_stamp": candidate["marker_stamp"],
                    "base_to_tool": candidate["base_to_tool"].copy(),
                    "camera_to_target": candidate["camera_to_target"].copy(),
                }
            )
            count = len(self.samples)
            try:
                write_yaml(self.session_dir / ("samples_" + timestamp() + ".yaml"),
                           {"stereo_binding": self.stereo_binding, "samples": self.samples})
            except OSError as exc:
                self.samples.pop()
                return TriggerResponse(False, "sample save failed: " + str(exc))
        return TriggerResponse(True, "captured sample {}".format(count))

    def delete_last(self, _request):
        with self.lock:
            if not self.samples:
                return TriggerResponse(False, "no samples to delete")
            self.samples.pop()
            count = len(self.samples)
        return TriggerResponse(True, "deleted last sample; {} remain".format(count))

    def reset(self, _request):
        with self.lock:
            self.samples = []
        return TriggerResponse(True, "all samples cleared")

    @staticmethod
    def _transform_record(parent, child, matrix):
        translation, quaternion = transform_to_xyz_quaternion(matrix)
        return {
            "parent_frame": parent,
            "child_frame": child,
            "translation_m": translation.tolist(),
            "quaternion_xyzw": quaternion.tolist(),
            "matrix_4x4": matrix_to_list(matrix),
        }

    def _lookup_link_to_optical(self):
        message = self.tf_buffer.lookup_transform(
            self.camera_link_frame,
            self.camera_frame,
            rospy.Time(0),
            rospy.Duration(2.0),
        )
        return transform_stamped_to_matrix(message)

    def _save_result(self, base_to_camera, link_to_optical, metrics, passed, samples):
        base_to_link = base_to_camera @ np.linalg.inv(link_to_optical)
        created_at = datetime.datetime.now(datetime.timezone.utc)
        result_stamp = created_at.strftime("%Y%m%dT%H%M%S.%fZ")
        os.makedirs(self.result_dir, exist_ok=True)
        result_path = os.path.join(
            str(self.session_dir), "{}_{}.yaml".format(result_stamp, self.serial)
        )
        result = {
            "schema_version": 1,
            "stereo_binding": self.stereo_binding,
            "physical_validation": "PENDING",
            "automatic_tf_publication": False,
            "status": "PASS" if passed else "FAIL",
            "created_at": created_at.isoformat(),
            "camera": {
                "name": self.camera,
                "namespace": "/{}".format(self.camera),
                "serial": self.serial,
            },
            "target": {
                "type": "single_aruco",
                "dictionary": "DICT_4X4_50",
                "marker_id": self.marker_id,
                "marker_size_m": self.marker_size_m,
            },
            "intrinsics": {
                "source": "calibrated_file",
                "file": self.intrinsics["path"],
                "sha256": self.intrinsics["sha256"],
                "source_record": self.intrinsics["source_record"],
                "camera": self.intrinsics["camera"],
                "camera_serial_number": self.intrinsics["serial"],
                "camera_model": self.intrinsics["camera_model"],
                "calibration_time": self.intrinsics["calibration_time"],
                "calibration_target": self.intrinsics["calibration_target"],
                "image_width": self.intrinsics["width"],
                "image_height": self.intrinsics["height"],
                "distortion_model": self.intrinsics["distortion_model"],
                "camera_matrix": self.intrinsics["camera_matrix"].tolist(),
                "distortion_coefficients": self.intrinsics["distortion"].tolist(),
                "rectification_matrix": self.intrinsics["rectification"].tolist(),
                "projection_matrix": self.intrinsics["projection"].tolist(),
            },
            "inputs": {
                "robot_pose_topic": self.robot_pose_topic,
                "image_topic": self.image_topic,
                "camera_info_topic": self.camera_info_topic,
                "marker_pose_topic": rospy.resolve_name(self.marker_pose_topic),
            },
            "solver": "OpenCV CALIB_HAND_EYE_TSAI (eye-to-hand)",
            "calibration": {
                "base_to_camera_optical": self._transform_record(
                    self.base_frame, self.camera_frame, base_to_camera
                ),
                "camera_optical_to_base": self._transform_record(
                    self.camera_frame, self.base_frame, np.linalg.inv(base_to_camera)
                ),
                "tool_to_target_mean": self._transform_record(
                    "tool", "aruco_marker_{}".format(self.marker_id), metrics["tool_to_target_mean"]
                ),
            },
            "tf_publication": {
                "base_to_camera_link": self._transform_record(
                    self.base_frame, self.camera_link_frame, base_to_link
                ),
                "camera_link_to_optical": self._transform_record(
                    self.camera_link_frame, self.camera_frame, link_to_optical
                ),
            },
            "quality": {
                "sample_count": len(samples),
                "translation_rms_m": metrics["translation_rms_m"],
                "translation_max_m": metrics["translation_max_m"],
                "rotation_rms_deg": metrics["rotation_rms_deg"],
                "rotation_max_deg": metrics["rotation_max_deg"],
                "limits": {
                    "translation_rms_m": self.translation_rms_limit_m,
                    "rotation_rms_deg": self.rotation_rms_limit_deg,
                },
                "passed": bool(passed),
                "note": "Internal consistency only; independent physical validation is required.",
            },
            "samples": [
                {
                    "index": index + 1,
                    "robot_stamp": sample["robot_stamp"],
                    "marker_stamp": sample["marker_stamp"],
                    "base_to_tool_matrix_4x4": matrix_to_list(sample["base_to_tool"]),
                    "camera_to_target_matrix_4x4": matrix_to_list(sample["camera_to_target"]),
                }
                for index, sample in enumerate(samples)
            ],
        }
        with open(result_path, "x", encoding="utf-8") as output:
            yaml.safe_dump(result, output, sort_keys=False, allow_unicode=True)
        return result_path

    def _check_binding(self):
        if sha256(self.intrinsics["path"]) != self.intrinsics["sha256"]:
            raise ValueError("intrinsics file changed; start a new handeye session")
        if sha256(self.stereo_path) != self.stereo_binding["sha256"]:
            raise ValueError("stereo result changed; start a new handeye session")

    def solve(self, _request):
        with self.lock:
            samples = [
                {
                    key: value.copy() if isinstance(value, np.ndarray) else value
                    for key, value in sample.items()
                }
                for sample in self.samples
            ]
        if len(samples) < self.min_samples:
            return TriggerResponse(
                False,
                "need at least {} samples; currently {}".format(self.min_samples, len(samples)),
            )

        try:
            self._check_binding()
            base_to_tool = [sample["base_to_tool"] for sample in samples]
            camera_to_target = [sample["camera_to_target"] for sample in samples]
            base_to_camera = solve_eye_to_hand(base_to_tool, camera_to_target)
            metrics = consistency_metrics(base_to_tool, camera_to_target, base_to_camera)
            link_to_optical = self._lookup_link_to_optical()
            if not np.allclose(link_to_optical, self.stereo["cameras"]["rs1"]["link_T_optical"], atol=1e-8):
                raise ValueError("RS1 link/optical transform changed since stereo calibration")
            passed = (
                metrics["translation_rms_m"] <= self.translation_rms_limit_m
                and metrics["rotation_rms_deg"] <= self.rotation_rms_limit_deg
            )
            result_path = self._save_result(
                base_to_camera, link_to_optical, metrics, passed, samples
            )
        except Exception as exception:
            rospy.logerr("calibration failed: %s", exception)
            return TriggerResponse(False, "calibration failed: {}".format(exception))

        message = "{}: {} (translation RMS {:.4f} m, rotation RMS {:.2f} deg)".format(
            "PASS" if passed else "FAIL",
            result_path,
            metrics["translation_rms_m"],
            metrics["rotation_rms_deg"],
        )
        rospy.loginfo(message) if passed else rospy.logwarn(message)
        return TriggerResponse(bool(passed), message)


def main():
    rospy.init_node("calibration_server")
    try:
        CalibrationServer()
        rospy.spin()
    except Exception as exception:
        rospy.logfatal("Calibration server startup failed: %s", exception)
        raise


if __name__ == "__main__":
    main()
