#!/usr/bin/env python3

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

from piper_eye_to_hand_calibration.aruco_support import (
    create_detector,
    detect_markers,
    estimate_marker_pose,
)
from piper_eye_to_hand_calibration.intrinsics import load_intrinsics
from piper_eye_to_hand_calibration.artifacts import sha256
from piper_eye_to_hand_calibration.stereo import bind_handeye
from piper_eye_to_hand_calibration.ros_support import (
    matrix_to_pose,
    normalize_frame,
    wait_for_camera_identity,
)


class ArucoDetectorNode:
    def __init__(self):
        self.camera = rospy.get_param("~camera")
        self.serial = wait_for_camera_identity(
            self.camera,
            rospy.get_param("~camera_serial"),
            rospy.get_param("~serial_check_timeout", 10.0),
        )
        self.marker_id = int(rospy.get_param("~marker_id", 18))
        self.marker_size_m = float(rospy.get_param("~marker_size_m", 0.100))
        self.max_reprojection_error_px = float(
            rospy.get_param("~max_reprojection_error_px", 2.0)
        )
        self.camera_frame = normalize_frame(
            rospy.get_param("~camera_frame", "{}_color_optical_frame".format(self.camera))
        )
        self.intrinsics = load_intrinsics(
            rospy.get_param("~intrinsics_file"),
            expected_camera=self.camera,
            expected_serial=self.serial,
        )
        self.reported_intrinsics = False
        bind_handeye(rospy.get_param("~stereo_result"), self.intrinsics)
        image_topic = rospy.get_param(
            "~image_topic", "/{}/color/image_raw".format(self.camera)
        )
        camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/{}/color/camera_info".format(self.camera)
        )

        if self.marker_id < 0 or self.marker_id >= 50:
            raise ValueError("DICT_4X4_50 marker_id must be in [0, 49]")
        if self.marker_size_m <= 0.0:
            raise ValueError("marker_size_m must be positive")

        self.bridge = CvBridge()
        self.dictionary, self.detector_parameters, self.detector = create_detector()
        self.pose_publisher = rospy.Publisher("marker_pose", PoseStamped, queue_size=1)
        self.debug_publisher = rospy.Publisher("debug_image", Image, queue_size=1)

        self.image_subscriber = message_filters.Subscriber(image_topic, Image)
        self.info_subscriber = message_filters.Subscriber(camera_info_topic, CameraInfo)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscriber, self.info_subscriber], queue_size=10, slop=0.03
        )
        self.synchronizer.registerCallback(self.callback)
        rospy.loginfo(
            "%s ArUco detector ready: DICT_4X4_50 id=%d size=%.3f m serial=%s intrinsics=%s",
            self.camera,
            self.marker_id,
            self.marker_size_m,
            self.serial,
            self.intrinsics["path"],
        )

    def callback(self, image_message, camera_info):
        if sha256(self.intrinsics["path"]) != self.intrinsics["sha256"]:
            rospy.logerr_throttle(2, "intrinsics file changed; restart calibration")
            return
        stamps = [image_message.header.stamp.to_sec(), camera_info.header.stamp.to_sec()]
        now = rospy.Time.now().to_sec()
        if any(s <= 0 or now-s > .5 or s-now > .1 for s in stamps):
            rospy.logwarn_throttle(2, "stale or future RGB/CameraInfo")
            return
        if normalize_frame(image_message.header.frame_id) != self.camera_frame:
            return
        info_frame = normalize_frame(camera_info.header.frame_id)
        if info_frame != self.camera_frame:
            rospy.logwarn_throttle(
                2.0,
                "CameraInfo frame mismatch: expected %s, observed %s",
                self.camera_frame,
                info_frame or "<empty>",
            )
            return
        if (
            int(camera_info.width) != self.intrinsics["width"]
            or int(camera_info.height) != self.intrinsics["height"]
        ):
            rospy.logerr_throttle(
                2.0,
                "Image resolution does not match calibrated intrinsics: expected %dx%d, got %dx%d",
                self.intrinsics["width"],
                self.intrinsics["height"],
                camera_info.width,
                camera_info.height,
            )
            return
        if camera_info.distortion_model != self.intrinsics["distortion_model"]:
            rospy.logerr_throttle(
                2.0,
                "CameraInfo distortion model mismatch: expected %s, got %s",
                self.intrinsics["distortion_model"],
                camera_info.distortion_model,
            )
            return

        try:
            image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", error)
            return
        if image.shape[1] != self.intrinsics["width"] or image.shape[0] != self.intrinsics["height"]:
            rospy.logerr_throttle(2.0, "Image dimensions do not match CameraInfo/intrinsics")
            return

        debug_image = image.copy()
        corners, identifiers, _ = detect_markers(
            image, self.dictionary, self.detector_parameters, self.detector
        )
        camera_matrix = self.intrinsics["camera_matrix"]
        distortion = self.intrinsics["distortion"]
        if not self.reported_intrinsics:
            live_matrix = np.asarray(camera_info.K, dtype=np.float64).reshape(3, 3)
            rospy.loginfo(
                "%s PnP uses calibrated file intrinsics; max |K_file-K_live|=%.6f px",
                self.camera,
                float(np.max(np.abs(camera_matrix - live_matrix))),
            )
            self.reported_intrinsics = True

        best = None
        if identifiers is not None:
            for marker_corners, identifier in zip(corners, np.asarray(identifiers).reshape(-1)):
                points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
                color = (0, 255, 0) if int(identifier) == self.marker_id else (0, 165, 255)
                cv2.polylines(debug_image, [points.astype(np.int32)], True, color, 2)
                cv2.putText(
                    debug_image,
                    "id={}".format(int(identifier)),
                    tuple(points[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )
                if int(identifier) != self.marker_id:
                    continue
                try:
                    transform, error, rotation_vector = estimate_marker_pose(
                        points, camera_matrix, distortion, self.marker_size_m
                    )
                except (ValueError, cv2.error) as exception:
                    rospy.logwarn_throttle(2.0, "marker pose estimation failed: %s", exception)
                    continue
                if best is None or error < best[1]:
                    best = (transform, error, rotation_vector)

        if best is not None:
            transform, error, rotation_vector = best
            accepted = error <= self.max_reprojection_error_px
            status_color = (0, 255, 0) if accepted else (0, 0, 255)
            cv2.putText(
                debug_image,
                "file K/D reprojection={:.2f}px {}".format(
                    error, "OK" if accepted else "REJECT"
                ),
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                status_color,
                2,
            )
            cv2.drawFrameAxes(
                debug_image,
                camera_matrix,
                distortion,
                rotation_vector,
                transform[:3, 3],
                self.marker_size_m * 0.5,
            )
            if accepted:
                pose_message = PoseStamped()
                pose_message.header.stamp = image_message.header.stamp
                pose_message.header.frame_id = self.camera_frame
                pose_message.pose = matrix_to_pose(transform)
                self.pose_publisher.publish(pose_message)
        else:
            cv2.putText(
                debug_image,
                "ArUco id {} not detected".format(self.marker_id),
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        try:
            debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_message.header = image_message.header
            self.debug_publisher.publish(debug_message)
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "debug image conversion failed: %s", error)


def main():
    rospy.init_node("aruco_detector")
    try:
        ArucoDetectorNode()
        rospy.spin()
    except Exception as exception:
        rospy.logfatal("ArUco detector startup failed: %s", exception)
        raise


if __name__ == "__main__":
    main()
