#!/usr/bin/env python3
"""Preserve and import one ROS camera_calibration archive."""

import argparse
from piper_eye_to_hand_calibration.artifacts import RESULT_ROOT, SERIALS, timestamp
from piper_eye_to_hand_calibration.intrinsics_import import import_archive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=list(SERIALS), required=True)
    parser.add_argument("--archive", default="/tmp/calibrationdata.tar.gz")
    parser.add_argument("--output")
    args = parser.parse_args()
    output = args.output or RESULT_ROOT / "intrinsics" / args.camera / timestamp()
    print(import_archive(args.archive, args.camera, output))


if __name__ == "__main__":
    main()
