"""Import the unmodified ROS monocular archive into an immutable session."""

import hashlib
import io
from pathlib import Path, PurePosixPath
import tarfile

import yaml

from .artifacts import IMAGE_SIZE, SERIALS, sha256, write_yaml
from .intrinsics import load_intrinsics


def import_archive(archive_path, camera, output):
    if camera not in SERIALS:
        raise ValueError("camera must be rs1 or rs3")
    raw = Path(archive_path).read_bytes()
    with tarfile.open(fileobj=io.BytesIO(raw), mode="r:gz") as archive:
        members = archive.getmembers()
        names = [m.name for m in members]
        if len(names) != len(set(names)):
            raise ValueError("duplicate archive member")
        for member in members:
            name = PurePosixPath(member.name)
            if (name.is_absolute() or ".." in name.parts or
                    not (member.isfile() or member.isdir())):
                raise ValueError("unsafe archive member: " + member.name)
        if "ost.yaml" not in names or not any(n.startswith("left-") and n.endswith(".png") for n in names):
            raise ValueError("ROS archive must contain ost.yaml and original left-*.png images")
        profile = yaml.safe_load(archive.extractfile("ost.yaml").read())
        expected_name = "{}_{}_640x480".format(camera, SERIALS[camera])
        if not isinstance(profile, dict) or profile.get("camera_name") != expected_name:
            raise ValueError("camera_name must be " + expected_name + "; wrong camera/serial archive")
        if (profile.get("image_width"), profile.get("image_height")) != IMAGE_SIZE:
            raise ValueError("real 640x480 calibration required; do not scale old intrinsics")
        # Validate matrices before creating the destination.
        from .intrinsics import validate_profile
        profile.update(camera=camera, camera_serial_number=SERIALS[camera])
        validate_profile(profile, camera, SERIALS[camera])
        output = Path(output).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=False)
        (output / "calibrationdata.tar.gz").write_bytes(raw)
        originals = output / "ros_originals"
        originals.mkdir()
        for member in members:
            dest = originals / member.name
            if member.isdir():
                dest.mkdir(parents=True, exist_ok=True)
            else:
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(archive.extractfile(member).read())
    profile.update(
        source_record=str(originals / "ost.yaml"),
        source_archive=str(output / "calibrationdata.tar.gz"),
        source_archive_sha256=hashlib.sha256(raw).hexdigest(),
        calibration_target="11x8 inner corners, 0.025 m square size",
        status="PENDING_VALIDATION", independently_validated=False,
    )
    target = output / "intrinsics.yaml"
    write_yaml(target, profile)
    load_intrinsics(target, camera, SERIALS[camera])
    manifest = {str(p.relative_to(output)): sha256(p) for p in sorted(output.rglob("*")) if p.is_file()}
    write_yaml(output / "manifest.yaml", {"camera": camera, "serial": SERIALS[camera], "sha256": manifest})
    return target
