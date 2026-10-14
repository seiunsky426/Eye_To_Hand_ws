"""Shared file conventions for the calibration-only workflow."""

from datetime import datetime, timezone
import hashlib
from pathlib import Path

import numpy as np
import yaml

SERIALS = {"rs1": "346522071783", "rs3": "934222070377"}
IMAGE_SIZE = (640, 480)
RESULT_ROOT = Path.home() / "Eye_To_Hand_ws" / "Calculation Results"


def timestamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def write_yaml(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(plain(value), sort_keys=False, allow_unicode=True))
    temporary.replace(path)


def read_yaml(path):
    value = yaml.safe_load(Path(path).read_bytes())
    if not isinstance(value, dict):
        raise ValueError("expected YAML mapping: " + str(path))
    return value


def camera_record(profile, link_to_optical):
    name = profile["camera"]
    return {
        "serial": profile["serial"], "optical_frame": name + "_color_optical_frame",
        "link_frame": name + "_link", "link_T_optical": plain(link_to_optical),
        "intrinsics_file": profile["path"], "intrinsics_sha256": profile["sha256"],
        "camera_info": {"width": profile["width"], "height": profile["height"],
                        "distortion_model": profile["distortion_model"],
                        "K": profile["camera_matrix"].ravel().tolist(),
                        "D": profile["distortion"].tolist()},
    }
