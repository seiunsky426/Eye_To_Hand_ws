#!/usr/bin/env python3
"""Compose passed stereo and bound RS1 handeye files; never publishes TF."""
import argparse
from piper_eye_to_hand_calibration.artifacts import RESULT_ROOT, timestamp
from piper_eye_to_hand_calibration.composition import compose_results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stereo", required=True)
    parser.add_argument("--handeye", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    output = args.output or RESULT_ROOT / "composed" / timestamp()
    compose_results(args.stereo, args.handeye, output)
    print("PASS (internal checks only): " + str(output))


if __name__ == "__main__":
    main()
