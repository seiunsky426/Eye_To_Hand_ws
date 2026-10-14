"""Explicit, file-only composition of stereo and RS1 handeye results."""

from pathlib import Path
import numpy as np

from .artifacts import SERIALS, read_yaml, sha256, write_yaml
from .stereo import load_stereo
from .transforms import validate_transform, transform_to_xyz_quaternion


def transform_record(parent, child, matrix):
    matrix = validate_transform(matrix)
    translation, quaternion = transform_to_xyz_quaternion(matrix)
    return {"parent_frame": parent, "child_frame": child, "matrix_4x4": matrix.tolist(),
            "translation_m": translation.tolist(), "quaternion_xyzw": quaternion.tolist()}


def checked_record(record, parent, child):
    if record["parent_frame"] != parent or record["child_frame"] != child:
        raise ValueError("coordinate frame mismatch: expected " + parent + " -> " + child)
    return validate_transform(record["matrix_4x4"])


def compose_results(stereo_path, handeye_path, output):
    stereo, stereo_hash = load_stereo(stereo_path)
    handeye = read_yaml(handeye_path)
    q = handeye["quality"]
    scores = [q["translation_rms_m"], q["rotation_rms_deg"]]
    if (handeye.get("schema_version") != 1 or handeye.get("status") != "PASS"
            or q.get("passed") is not True or q["sample_count"] < 15
            or not np.all(np.isfinite(scores)) or min(scores) < 0
            or scores[0] > .010 or scores[1] > 1.5):
        raise ValueError("RS1 handeye internal consistency failed")
    if handeye["camera"]["name"] != "rs1" or handeye["camera"]["serial"] != SERIALS["rs1"]:
        raise ValueError("RS1 handeye camera identity mismatch")
    target = handeye.get("target", {})
    if target.get("dictionary") != "DICT_4X4_50" or target.get("marker_id") != 18 or target.get("marker_size_m") != .100:
        raise ValueError("handeye marker identity/size mismatch")
    if handeye.get("stereo_binding", {}).get("sha256") != stereo_hash:
        raise ValueError("handeye was not bound to this stereo result")
    info = handeye["intrinsics"]
    expected = stereo["cameras"]["rs1"]
    if (info.get("sha256") != expected["intrinsics_sha256"]
            or info.get("camera_serial_number") != SERIALS["rs1"]
            or (info.get("image_width"), info.get("image_height")) != (640, 480)
            or not np.allclose(np.asarray(info["camera_matrix"]).ravel(), expected["camera_info"]["K"])
            or not np.allclose(info["distortion_coefficients"], expected["camera_info"]["D"])):
        raise ValueError("handeye/stereo intrinsics mismatch")
    base_rs1 = checked_record(handeye["calibration"]["base_to_camera_optical"],
                              "arm_base", "rs1_color_optical_frame")
    internal = checked_record(handeye["tf_publication"]["camera_link_to_optical"],
                              "rs1_link", "rs1_color_optical_frame")
    if not np.allclose(internal, expected["link_T_optical"], atol=1e-8):
        raise ValueError("RS1 link/optical transform changed")
    saved_link = checked_record(handeye["tf_publication"]["base_to_camera_link"], "arm_base", "rs1_link")
    if not np.allclose(saved_link @ internal, base_rs1, atol=1e-8):
        raise ValueError("handeye link/optical composition inconsistent")
    sources = {
        "stereo": {"path": str(Path(stereo_path).resolve()), "sha256": stereo_hash,
                   "validation": stereo["validation"]},
        "handeye": {"path": str(Path(handeye_path).resolve()), "sha256": sha256(handeye_path), "quality": q},
    }
    transforms = {"rs1": base_rs1, "rs3": base_rs1 @ np.asarray(stereo["rs1_optical_T_rs3_optical"])}
    results = {}
    for name, matrix in transforms.items():
        camera = stereo["cameras"][name]
        link = np.asarray(camera["link_T_optical"])
        results[name] = {
            "schema_version": 1, "status": "PASS", "camera": {"name": name, "serial": SERIALS[name]},
            "intrinsics": {"source": "calibrated_file", "file": camera["intrinsics_file"],
                           "sha256": camera["intrinsics_sha256"], "camera": name,
                           "camera_serial_number": SERIALS[name], **camera["camera_info"]},
            "calibration": {"base_to_camera_optical": transform_record("arm_base", camera["optical_frame"], matrix),
                            "camera_optical_to_base": transform_record(camera["optical_frame"], "arm_base", np.linalg.inv(matrix))},
            "tf_publication": {"base_to_camera_link": transform_record("arm_base", camera["link_frame"], matrix @ np.linalg.inv(link)),
                               "camera_link_to_optical": transform_record(camera["link_frame"], camera["optical_frame"], link)},
            "sources": sources, "quality": {"passed": True, "note": "Internal checks only; physical accuracy unverified."},
            "physical_validation": "PENDING", "automatic_tf_publication": False,
        }
    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    for name, value in results.items():
        write_yaml(output / (name + "_extrinsics.yaml"), value)
    write_yaml(output / "composition.yaml", {"status": "PASS", "sources": sources,
               "formula": "arm_base_T_rs3_optical = arm_base_T_rs1_optical @ rs1_optical_T_rs3_optical",
               "physical_validation": "PENDING", "automatic_tf_publication": False})
    return results
