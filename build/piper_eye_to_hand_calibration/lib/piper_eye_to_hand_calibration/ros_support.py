"""Small ROS conversion and RealSense identity helpers."""

import time

import numpy as np
import rospy
from geometry_msgs.msg import Pose, TransformStamped

from .transforms import transform_from_xyz_quaternion, transform_to_xyz_quaternion


def normalize_frame(frame):
    return str(frame).lstrip("/")


def normalize_serial(serial):
    return str(serial).strip().lstrip("_")


def wait_for_camera_identity(camera, expected_serial, timeout_sec=10.0):
    camera = str(camera).strip().strip("/")
    expected = normalize_serial(expected_serial)
    supported_cameras = ("rs1", "rs3")
    if camera not in supported_cameras:
        raise ValueError("camera must be rs1 or rs3")
    if not expected:
        raise ValueError("camera_serial must not be empty")

    parameter = "/{}/realsense2_camera/serial_no".format(camera)
    deadline = time.monotonic() + float(timeout_sec)
    while not rospy.is_shutdown() and not rospy.has_param(parameter):
        if time.monotonic() >= deadline:
            raise RuntimeError("RealSense serial parameter not found: {}".format(parameter))
        rospy.sleep(0.1)

    observed = normalize_serial(rospy.get_param(parameter, ""))
    if not observed:
        raise RuntimeError("RealSense driver has an empty serial_no: {}".format(parameter))
    if observed != expected:
        raise RuntimeError(
            "{} serial mismatch: expected {}, observed {}".format(camera, expected, observed)
        )

    for other in supported_cameras:
        if other == camera:
            continue
        other_parameter = "/{}/realsense2_camera/serial_no".format(other)
        if rospy.has_param(other_parameter):
            other_serial = normalize_serial(rospy.get_param(other_parameter, ""))
            if other_serial and other_serial == observed:
                raise RuntimeError(
                    "{} and {} report the same serial {}".format(camera, other, observed)
                )
    return observed


def pose_to_matrix(pose):
    if not isinstance(pose, Pose):
        raise TypeError("expected geometry_msgs/Pose")
    return transform_from_xyz_quaternion(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
    )


def transform_stamped_to_matrix(message):
    transform = message.transform
    return transform_from_xyz_quaternion(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
    )


def matrix_to_pose(matrix):
    translation, quaternion = transform_to_xyz_quaternion(matrix)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = translation
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    return pose


def matrix_to_transform_stamped(matrix, parent_frame, child_frame, stamp=None):
    translation, quaternion = transform_to_xyz_quaternion(matrix)
    message = TransformStamped()
    message.header.stamp = rospy.Time.now() if stamp is None else stamp
    message.header.frame_id = normalize_frame(parent_frame)
    message.child_frame_id = normalize_frame(child_frame)
    message.transform.translation.x = translation[0]
    message.transform.translation.y = translation[1]
    message.transform.translation.z = translation[2]
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    return message
