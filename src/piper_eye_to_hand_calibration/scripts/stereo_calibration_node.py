#!/usr/bin/env python3
"""Two-view manual-origin 5+1 capture. Camera subscribers only; no robot IO."""

from copy import deepcopy
import json
from pathlib import Path
import threading
import time

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from piper_eye_to_hand_calibration.artifacts import (
    IMAGE_SIZE, RESULT_ROOT, SERIALS, camera_record, sha256, timestamp, write_yaml)
from piper_eye_to_hand_calibration.intrinsics import load_intrinsics
from piper_eye_to_hand_calibration.ros_support import wait_for_camera_identity, transform_stamped_to_matrix
from piper_eye_to_hand_calibration.stereo import (
    CAMERAS, TARGET, board_pose, check_frame_stamps, check_origin_handedness,
    check_stationary, detect_corners, ordered_corners, solve_dataset)


class StereoCapture:
    def __init__(self):
        self.profiles = {n: load_intrinsics(rospy.get_param("~"+n+"_intrinsics"), n, SERIALS[n]) for n in CAMERAS}
        for name, profile in self.profiles.items():
            if (profile["width"], profile["height"]) != IMAGE_SIZE:
                raise ValueError(name + ": real 640x480 intrinsics required")
            wait_for_camera_identity(name, SERIALS[name])
        self.root = Path(rospy.get_param("~output_directory", str(RESULT_ROOT / "stereo" / timestamp()))).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=False)
        self.cameras = {}
        self.driver_infos = {}
        self.lock = threading.Lock()
        self.latest = None
        self.last_processed = None
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.samples, self.frames = [], None
        self.origins, self.anchors, self.views = {}, {}, {}
        self.images = {}
        self.last_failure = ""
        self.attempt = None
        self.reason = "Click SAME physical inner corner in BOTH views. SPACE=capture R=reselect Q=quit"
        self.window = "RS1 | RS3 - 5 fit + 1 validation"
        self.subscribers = []
        for name in CAMERAS:
            self.subscribers.extend([
                message_filters.Subscriber("/"+name+"/color/image_raw", Image, queue_size=2),
                message_filters.Subscriber("/"+name+"/color/camera_info", CameraInfo, queue_size=2)])
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subscribers, 10, .080)
        self.sync.registerCallback(self.receive)

    def receive(self, *messages):
        with self.lock:
            self.latest = (messages, time.monotonic())

    def reject(self, reason):
        try:
            if self.attempt is not None:
                write_yaml(self.attempt / "failure.yaml", {"reason": reason, "saved_pairs": len(self.frames or []),
                           "origins": self.origins, "time": timestamp()})
            if reason != self.last_failure:
                with (self.root / "events.jsonl").open("a") as log:
                    log.write(json.dumps({"time": timestamp(), "reason": reason}) + "\n")
        except OSError as exc:
            rospy.logerr("Could not persist capture failure (%s): %s", reason, exc)
        if reason != self.last_failure:
            rospy.logwarn(reason)
        self.last_failure = reason
        self.frames, self.attempt = None, None
        self.origins, self.anchors, self.views = {}, {}, {}
        self.reason = reason + "; reselect both origins"

    def click(self, event, x, y, _flags, _parameter):
        if event != cv2.EVENT_LBUTTONDOWN or self.frames is not None or len(self.samples) == 6:
            return
        name = CAMERAS[0] if x < 640 else CAMERAS[1]
        x = x if x < 640 else x - 640
        if y >= 480 or name not in self.views:
            return
        view = self.views[name]
        try:
            if time.monotonic() - view["processed"] > .5:
                raise ValueError("displayed preview expired")
            corners = ordered_corners(view["raw_corners"], [x, y])
            pose, error = board_pose(corners, self.cameras[name]["camera_info"])
            if error > .8:
                raise ValueError("PnP RMS exceeds 0.8 px")
            self.origins[name] = {"point_px": corners[0].tolist(), "click_px": [x, y], "stamp_s": view["stamp"]}
            self.anchors[name] = {"corners_px": corners, "camera_info": self.cameras[name]["camera_info"], "camera_T_board": pose}
            if len(self.anchors) == 2:
                check_origin_handedness(self.anchors)
            self.reason = "Origin selected: " + ", ".join(self.origins) + ". Check +X/+Y; SPACE=capture"
        except (ValueError, cv2.error) as exc:
            self.reject(str(exc))

    def process(self, messages):
        stamps = [messages[i].header.stamp.to_sec() for i in (0, 2)]
        info_stamps = [messages[i].header.stamp.to_sec() for i in (1, 3)]
        check_frame_stamps(stamps, info_stamps, rospy.Time.now().to_sec(),
                           self.frames[-1]["stamps_s"] if self.frames else None)
        observations, views = {}, {}
        self.images = {name: self.bridge.imgmsg_to_cv2(messages[2*i], "bgr8")
                       for i, name in enumerate(CAMERAS)}
        for index, name in enumerate(CAMERAS):
            image_msg, info = messages[2*index:2*index+2]
            profile = self.profiles[name]
            if sha256(profile["path"]) != profile["sha256"]:
                raise ValueError(name + " intrinsics changed; start new session")
            if str(rospy.get_param("/"+name+"/realsense2_camera/serial_no", "")).lstrip("_") != SERIALS[name]:
                raise ValueError(name + " camera identity changed")
            if (image_msg.header.frame_id != name+"_color_optical_frame" or info.header.frame_id != image_msg.header.frame_id
                    or (info.width, info.height) != IMAGE_SIZE or info.distortion_model != "plumb_bob"):
                raise ValueError(name + " frame/resolution/distortion mismatch")
            live = {"K": list(info.K), "D": list(info.D), "R": list(info.R), "P": list(info.P)}
            if name in self.driver_infos and live != self.driver_infos[name]:
                raise ValueError(name + " driver CameraInfo changed; start new session")
            self.driver_infos[name] = live
            internal = transform_stamped_to_matrix(self.tf_buffer.lookup_transform(
                name+"_link", name+"_color_optical_frame", rospy.Time(0), rospy.Duration(.2)))
            if name in self.cameras and not np.allclose(internal, self.cameras[name]["link_T_optical"], atol=1e-8):
                raise ValueError(name + " internal TF changed; start new session")
            self.cameras[name] = camera_record(profile, internal)
            image = self.images[name]
            if self.attempt is not None:
                # Save attempted images even when corner detection subsequently fails.
                if not cv2.imwrite(str(self.attempt / ("{:02d}_{}.png".format(len(self.frames), name))), image):
                    raise OSError("image save failed")
            raw = detect_corners(image)
            view = {"raw_corners": raw, "image": image, "stamp": stamps[index], "processed": time.monotonic()}
            if name in self.origins:
                corners = ordered_corners(raw, self.origins[name]["point_px"])
                pose, error = board_pose(corners, self.cameras[name]["camera_info"])
                if error > .8:
                    raise ValueError(name + " PnP RMS exceeds 0.8 px")
                observations[name] = {"corners_px": corners, "camera_T_board": pose,
                                      "camera_info": self.cameras[name]["camera_info"], "pnp_rms_px": error}
                if self.attempt is not None:
                    observations[name]["image_path"] = str(self.attempt / ("{:02d}_{}.png".format(len(self.frames), name)))
                check_stationary({name: self.anchors[name]}, {name: observations[name]})
                view["corners"] = corners
            views[name] = view
        if len(observations) == 2:
            check_origin_handedness(observations)
        self.views = views
        self.last_failure = ""
        if self.frames is None:
            return
        check_frame_stamps(stamps, info_stamps, rospy.Time.now().to_sec())
        frame = {"stamps_s": stamps, "camera_info_stamps_s": info_stamps,
                 "driver_camera_info": self.driver_infos, "cameras": observations}
        write_yaml(self.attempt / ("{:02d}.yaml".format(len(self.frames))), frame)
        self.frames.append(frame)
        self.reason = "S{:02d}: saved {}/30 pairs; keep board still".format(len(self.samples)+1, len(self.frames))
        if len(self.frames) == 30:
            self.finish_placement()

    def start_capture(self):
        if len(self.samples) == 6 or self.frames is not None:
            return
        if len(self.origins) != 2 or len(self.views) != 2:
            raise ValueError("select both origins first")
        if any(time.monotonic()-v["processed"] > .5 for v in self.views.values()):
            raise ValueError("preview expired")
        check_origin_handedness(self.anchors)
        self.attempt = self.root / ("S{:02d}_".format(len(self.samples)+1) + timestamp())
        self.attempt.mkdir(exist_ok=False)
        self.frames = []
        self.capture_started = time.monotonic()
        write_yaml(self.attempt / "selection.yaml", {"origins": self.origins})

    def finish_placement(self):
        number = len(self.samples)+1
        sample = {"id": number, "split": "validation" if number == 6 else "optimization",
                  "origins": deepcopy(self.origins), "attempt": str(self.attempt), "frames": self.frames,
                  "cameras": {n: {"corners_px": np.median([f["cameras"][n]["corners_px"] for f in self.frames], axis=0)} for n in CAMERAS}}
        for previous in self.samples:
            for name in CAMERAS:
                delta = sample["cameras"][name]["corners_px"] - previous["cameras"][name]["corners_px"]
                if np.sqrt(np.mean(np.sum(delta**2, axis=1))) < 3:
                    raise ValueError("duplicate placement; move or turn board")
        dataset = {"schema_version": 1, "target": TARGET, "complete": number == 6,
                   "cameras": self.cameras, "samples": self.samples + [sample]}
        write_yaml(self.root / "dataset.yaml", dataset)
        self.samples.append(sample)
        self.frames, self.attempt = None, None
        self.origins, self.anchors, self.views = {}, {}, {}
        self.reason = "S{:02d} saved; move board and select both origins again".format(number)
        if number == 6:
            try:
                report = solve_dataset(dataset)
                report.update(dataset_path=str(self.root / "dataset.yaml"), dataset_sha256=sha256(self.root / "dataset.yaml"))
                write_yaml(self.root / "stereo_result.yaml", report)
                self.reason = report["status"] + ": " + str(self.root / "stereo_result.yaml")
                rospy.loginfo(self.reason)
            except (ValueError, cv2.error) as exc:
                write_yaml(self.root / "solve_failure.yaml", {"reason": str(exc)})
                self.reason = "SOLVE_FAILED: " + str(exc)

    def run(self):
        # AUTOSIZE keeps mouse coordinates identical to the original 640x480 pixels.
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window, self.click)
        try:
            while not rospy.is_shutdown():
                with self.lock:
                    latest = self.latest
                if len(self.samples) < 6:
                    try:
                        if latest is None or time.monotonic()-latest[1] > .5:
                            if self.origins or self.frames is not None:
                                raise ValueError("camera stream expired")
                        else:
                            key = tuple(m.header.stamp.to_nsec() for m in latest[0])
                            if key != self.last_processed:
                                self.last_processed = key
                                self.process(latest[0])
                            elif self.views and any(time.monotonic()-v["processed"] > .5 for v in self.views.values()):
                                raise ValueError("repeated/stalled images")
                        if self.frames is not None and time.monotonic()-self.capture_started > 45:
                            raise ValueError("30-pair capture timed out")
                    except (ValueError, OSError, cv2.error, tf2_ros.TransformException) as exc:
                        self.reject(str(exc))
                canvas = np.zeros((540, 1280, 3), np.uint8)
                for index, name in enumerate(CAMERAS):
                    if name not in self.images:
                        continue
                    if name not in self.views:
                        im = cv2.resize(self.images[name], IMAGE_SIZE)
                        cv2.putText(im, name + " - no valid selection", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                        canvas[:480, index*640:(index+1)*640] = im
                        continue
                    view = self.views[name]
                    im = view["image"].copy()
                    corners = view.get("corners", view["raw_corners"])
                    cv2.drawChessboardCorners(im, (11, 8), corners.astype(np.float32).reshape(-1, 1, 2), True)
                    if name in self.origins:
                        corners = ordered_corners(view["raw_corners"], self.origins[name]["point_px"])
                        p0 = tuple(np.rint(corners[0]).astype(int))
                        for endpoint, label, color in ((10, "+X", (0, 0, 255)), (77, "+Y", (0, 255, 0))):
                            end = tuple(np.rint(corners[endpoint]).astype(int))
                            cv2.arrowedLine(im, p0, end, color, 2)
                            label_at = tuple(np.rint((corners[0]+corners[endpoint])/2 + [8, -8]).astype(int))
                            cv2.putText(im, label, label_at, cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 4)
                            cv2.putText(im, label, label_at, cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
                        for point_id in (0, 10, 77, 87):
                            point = tuple(np.rint(corners[point_id] + [5, 18]).astype(int))
                            label = "O(0)" if point_id == 0 else str(point_id)
                            cv2.putText(im, label, point, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
                            cv2.putText(im, label, point, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
                    cv2.putText(im, name, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
                    canvas[:480, index*640:(index+1)*640] = im
                cv2.putText(canvas, "Saved: {}/6 (S06 validation) SPACE=capture R=reselect Q=quit".format(len(self.samples)), (10, 502), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
                cv2.putText(canvas, self.reason[:150], (10, 527), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
                cv2.imshow(self.window, canvas)
                key = cv2.waitKey(20) & 255
                if key in (ord("q"), 27) or cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key == ord("r") and len(self.samples) < 6:
                    self.reject("operator reset selection")
                if key == ord(" "):
                    try:
                        self.start_capture()
                    except ValueError as exc:
                        self.reject(str(exc))
        finally:
            if self.frames is not None:
                self.reject("session closed during capture")
            cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("stereo_calibration")
    StereoCapture().run()
