"""Manual-origin 5+1 stereo geometry, adapted from eyetohandauto_ws.

A_T_B maps coordinates in B to coordinates in A. Clicks only select order.
"""
import cv2
import numpy as np

from .artifacts import IMAGE_SIZE, SERIALS, read_yaml, sha256
from .transforms import rotation_distance_deg, validate_transform

CalibrationFailure = ValueError
CAMERAS = tuple(SERIALS)
TARGET = {"pattern_cols": 11, "pattern_rows": 8, "square_size_m": .025,
          "orientation_mode": "manual_origin", "x_direction": "11_point_edge_from_origin",
          "y_direction": "8_point_edge_from_origin", "z_direction": "right_hand_rule"}
LIMITS = {"reprojection_rmse_px": .8, "median_m": .005, "p95_m": .008}

def object_points():
    points = np.zeros((88, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:11, 0:8].T.reshape(-1, 2) * .025
    return points


def ordered_corners(corners, origin_px, maximum_click_distance_px=35.0):
    """Select only a lattice vertex; click coordinates never enter the fit."""
    grid = np.asarray(corners, dtype=float).reshape(8, 11, 2)
    click = np.asarray(origin_px, dtype=float).reshape(2)
    if not np.all(np.isfinite(grid)) or not np.all(np.isfinite(click)):
        raise CalibrationFailure("nonfinite corner or origin")
    distance = np.linalg.norm(grid.reshape(-1, 2)-click, axis=1)
    nearest = int(np.argmin(distance))
    if nearest not in (0, 10, 77, 87) or distance[nearest] > maximum_click_distance_px:
        raise CalibrationFailure("select one of the four inner-corner lattice vertices")
    row, column = divmod(nearest, 11)
    if row == 7:
        grid = grid[::-1]
    if column == 10:
        grid = grid[:, ::-1]
    return grid.reshape(88, 2).copy()


def camera_parameters(info):
    matrix = np.asarray(info["K"], dtype=float).reshape(3, 3)
    distortion = np.asarray(info.get("D", []), dtype=float)
    if (not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion))
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or info.get("distortion_model") not in ("plumb_bob", "rational_polynomial")):
        raise CalibrationFailure("invalid or unsupported RGB intrinsics")
    return matrix, distortion


def project(pose, info):
    matrix, distortion = camera_parameters(info)
    return cv2.projectPoints(object_points(), cv2.Rodrigues(pose[:3, :3])[0],
                             pose[:3, 3], matrix, distortion)[0].reshape(88, 2)


def board_pose(corners, info):
    matrix, distortion = camera_parameters(info)
    result = cv2.solvePnPGeneric(object_points(), np.asarray(corners, dtype=float),
                                matrix, distortion, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for rvec, tvec in zip(result[1], result[2]):
        pose = np.eye(4)
        pose[:3, :3] = cv2.Rodrigues(rvec)[0]
        pose[:3, 3] = np.asarray(tvec).reshape(3)
        xyz = (pose[:3, :3] @ object_points().T).T + pose[:3, 3]
        if np.all(xyz[:, 2] > 0):
            error = np.linalg.norm(project(pose, info)-corners, axis=1)
            candidates.append((float(np.sqrt(np.mean(error**2))), pose))
    candidates.sort(key=lambda item: item[0])
    if not candidates:
        raise CalibrationFailure("no positive-depth checkerboard pose")
    if len(candidates) > 1:
        best, other = candidates[:2]
        distinct = (np.linalg.norm(best[1][:3, 3]-other[1][:3, 3]) > 1e-5
                    or rotation_distance_deg(best[1], other[1]) > .001)
        if distinct and best[0] / max(other[0], 1e-12) > .95:
            raise CalibrationFailure("ambiguous planar PnP pose")
    return candidates[0][1], candidates[0][0]


def check_frame_stamps(stamps, info_stamps, now_ros, previous=None):
    values = np.asarray(list(stamps)+list(info_stamps), dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)) or np.any(values <= 0):
        raise CalibrationFailure("invalid RGB/CameraInfo timestamp")
    ages = now_ros-values
    if np.any(ages > .25):
        raise CalibrationFailure("stale RGB/CameraInfo: maximum age {:.1f} ms".format(
            float(np.max(ages))*1000.))
    leads = values-now_ros
    if np.any(leads > .080 + 1e-9):
        raise CalibrationFailure("future RGB/CameraInfo: maximum lead {:.1f} ms".format(
            float(np.max(leads))*1000.))
    span = float(np.ptp(values))
    if span > .080 + 1e-9:
        raise CalibrationFailure("unsynchronized RGB/CameraInfo: span {:.1f} ms".format(
            span*1000.))
    if previous is not None and np.any(np.asarray(stamps) <= previous):
        raise CalibrationFailure("repeated or out-of-order RGB images")


def check_stationary(reference, current):
    if not reference or set(reference) != set(current):
        raise CalibrationFailure("stationary observation cameras changed")
    for name in reference:
        first, last = reference[name], current[name]
        if first["camera_info"] != last["camera_info"]:
            raise CalibrationFailure(name+" CameraInfo changed")
        if np.max(np.linalg.norm(np.asarray(first["corners_px"])
                                 -last["corners_px"], axis=1)) > 1.5:
            raise CalibrationFailure(name+" board moved or corner order changed")
        a, b = np.asarray(first["camera_T_board"]), np.asarray(last["camera_T_board"])
        if (np.linalg.norm(a[:3, 3]-b[:3, 3]) > .0015
                or rotation_distance_deg(a, b) > .15):
            raise CalibrationFailure(name+" board pose changed during capture")


def detect_corners(image):
    if tuple(image.shape[1::-1]) != IMAGE_SIZE:
        raise ValueError("640x480 RGB required")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray, (11, 8), flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
    if not found or corners is None or len(corners) != 88:
        raise ValueError("88 inner corners not detected")
    points = corners.reshape(88, 2).astype(float)
    grid = points.reshape(8, 11, 2)
    spacing = np.r_[np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(),
                    np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel()]
    if np.min(spacing) < 6 or np.median(spacing) < 8:
        raise ValueError("checkerboard cells too small (minimum 6 px, median 8 px)")
    if np.any(points < 10) or np.any(points > np.asarray(IMAGE_SIZE) - 10):
        raise ValueError("checkerboard too close to image boundary")
    return points


def check_origin_handedness(observations):
    signs = []
    for name in CAMERAS:
        p = np.asarray(observations[name]["corners_px"])
        signs.append(np.sign(np.cross(p[10]-p[0], p[77]-p[0])))
    if signs[0] == 0 or signs[0] != signs[1]:
        raise ValueError("origin directions disagree between the two printed-face views")


def validate_camera(name, record):
    if (record["serial"] != SERIALS[name]
            or record["optical_frame"] != name + "_color_optical_frame"
            or record["link_frame"] != name + "_link"
            or (record["camera_info"]["width"], record["camera_info"]["height"]) != IMAGE_SIZE):
        raise ValueError(name + " identity, frames or resolution mismatch")
    camera_parameters(record["camera_info"])
    validate_transform(record["link_T_optical"])
    if len(record.get("intrinsics_sha256", "")) != 64:
        raise ValueError(name + " missing intrinsics hash")


def solve_dataset(dataset):
    if (dataset.get("schema_version") != 1 or dataset.get("target") != TARGET
            or dataset.get("complete") is not True):
        raise ValueError("complete 5+1 dataset required")
    samples = dataset["samples"]
    if [s["id"] for s in samples] != list(range(1, 7)):
        raise ValueError("exactly S01..S06 required")
    for name in CAMERAS:
        validate_camera(name, dataset["cameras"][name])
    for i, sample in enumerate(samples):
        if sample["split"] != ("validation" if i == 5 else "optimization"):
            raise ValueError("fixed 5+1 split changed")
        for name in CAMERAS:
            points = np.asarray(sample["cameras"][name]["corners_px"])
            if points.shape != (88, 2) or not np.all(np.isfinite(points)):
                raise ValueError("invalid checkerboard corners")
            for earlier in samples[:i]:
                delta = points - np.asarray(earlier["cameras"][name]["corners_px"])
                if np.sqrt(np.mean(np.sum(delta**2, axis=1))) < 3:
                    raise ValueError("duplicate placement: move or rotate the board")
        check_origin_handedness(sample["cameras"])
    infos = {n: dataset["cameras"][n]["camera_info"] for n in CAMERAS}
    k1, d1 = camera_parameters(infos["rs1"])
    k3, d3 = camera_parameters(infos["rs3"])
    output = cv2.stereoCalibrate(
        [object_points().astype(np.float32) for _ in samples[:5]],
        [np.asarray(s["cameras"]["rs1"]["corners_px"], np.float32) for s in samples[:5]],
        [np.asarray(s["cameras"]["rs3"]["corners_px"], np.float32) for s in samples[:5]],
        k1.copy(), d1.copy(), k3.copy(), d3.copy(), IMAGE_SIZE,
        flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-9))
    inverse = np.eye(4)
    inverse[:3, :3], inverse[:3, 3] = output[5], output[6].reshape(3)
    relative = validate_transform(np.linalg.inv(inverse))
    holdout = samples[5]["cameras"]
    poses = {n: board_pose(holdout[n]["corners_px"], infos[n])[0] for n in CAMERAS}
    predicted = {"rs1": relative @ poses["rs3"], "rs3": inverse @ poses["rs1"]}
    rms = {n: float(np.sqrt(np.mean(np.sum(
        (project(predicted[n], infos[n]) - holdout[n]["corners_px"])**2, axis=1)))) for n in CAMERAS}
    points = np.c_[object_points(), np.ones(88)]
    delta = ((poses["rs1"] @ points.T).T - (relative @ poses["rs3"] @ points.T).T)[:, :3]
    distances = np.linalg.norm(delta, axis=1)
    median, p95 = float(np.median(distances)), float(np.percentile(distances, 95))
    passed = max(rms.values()) <= .8 and median <= .005 and p95 <= .008
    return {
        "schema_version": 1, "workflow": "stereo_5plus1", "status": "PASS" if passed else "FAIL",
        "target": TARGET, "cameras": dataset["cameras"], "training_ids": [1, 2, 3, 4, 5],
        "validation_ids": [6], "rs1_optical_T_rs3_optical": relative.tolist(),
        "rs3_optical_T_rs1_optical": inverse.tolist(),
        "transform_convention": "A_T_B maps B coordinates into A",
        "training_reprojection_rms_px": float(output[0]),
        "validation": {"passed": bool(passed), "limits": LIMITS,
                       "cross_reprojection_rms_px": rms, "rgb_pnp_median_m": median,
                       "rgb_pnp_p95_m": p95, "rgb_pnp_corner_distances_m": distances.tolist()},
        "validation_scope": "one_independent_placement_RGB_PnP_only",
        "physical_validation": "PENDING", "depth_verified": False,
        "automatic_tf_publication": False,
    }


def load_stereo(path, expected_hash=None):
    report = read_yaml(path)
    digest = sha256(path)
    if expected_hash is not None and digest != expected_hash:
        raise ValueError("bound stereo result changed")
    if (report.get("schema_version") != 1 or report.get("workflow") != "stereo_5plus1"
            or report.get("target") != TARGET or report.get("training_ids") != [1, 2, 3, 4, 5]
            or report.get("validation_ids") != [6]):
        raise ValueError("a 5+1 stereo result is required")
    for name in CAMERAS:
        validate_camera(name, report["cameras"][name])
    v = report["validation"]
    scores = [v["cross_reprojection_rms_px"][n] for n in CAMERAS] + [v["rgb_pnp_median_m"], v["rgb_pnp_p95_m"]]
    if (report.get("status") != "PASS" or v.get("passed") is not True
            or not np.all(np.isfinite(scores)) or min(scores) < 0
            or np.any(np.asarray(scores) > [.8, .8, .005, .008])):
        raise ValueError("stereo validation failed; composition refused")
    relative = validate_transform(report["rs1_optical_T_rs3_optical"])
    inverse = validate_transform(report["rs3_optical_T_rs1_optical"])
    if not np.allclose(relative @ inverse, np.eye(4), atol=1e-8):
        raise ValueError("stereo inverse transform mismatch")
    return report, digest


def bind_handeye(stereo_path, profile):
    report, digest = load_stereo(stereo_path)
    source = report["cameras"]["rs1"]
    if (profile["camera"] != "rs1" or profile["serial"] != SERIALS["rs1"]
            or (profile["width"], profile["height"]) != IMAGE_SIZE
            or profile["sha256"] != source["intrinsics_sha256"]
            or not np.allclose(profile["camera_matrix"].ravel(), source["camera_info"]["K"])
            or not np.allclose(profile["distortion"], source["camera_info"]["D"])):
        raise ValueError("RS1 handeye intrinsics differ from stereo stage")
    return report, {"path": str(stereo_path), "sha256": digest}


