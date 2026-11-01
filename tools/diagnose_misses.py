#!/usr/bin/env python3
"""Shows what the classifier saw at the location of each missed object, and saves crops of misses.

This only reports. It doesn't change any score.
"""
import argparse
import collections
import json
import pathlib
import sys

import cv2

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "vision"))
sys.path.insert(0, str(ROOT / "tools"))

from classifier import Classifier  # noqa: E402
from score_recognition import box_area, counts, inter_area, is_hit  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=pathlib.Path)
    parser.add_argument("--classes", default="animal,traffic_control,obstruction")
    parser.add_argument("--stride", type=int, default=1)
    args = parser.parse_args()

    wanted = set(args.classes.split(","))
    classifier = Classifier(ROOT / "models")
    found_instead = {c: collections.Counter() for c in wanted}
    sizes = {c: {"hit": [], "miss": []} for c in wanted}
    false_alarms = collections.Counter()
    crops = {c: [] for c in wanted}

    lines = (args.frames / "ground_truth.jsonl").read_text().splitlines()
    for line in lines[:: args.stride]:
        record = json.loads(line)
        frame = cv2.imread(str(args.frames / record["frame"]))
        result = classifier.classify(frame)
        for obj in record["objects"]:
            for d in result.detections:
                # Detections of a class with no matching object of that class, by source.
                pass
            if obj["class"] not in wanted or not counts(obj):
                continue
            height = obj["box"][3] - obj["box"][1]
            if any(is_hit(obj, d) for d in result.detections):
                sizes[obj["class"]]["hit"].append(height)
                continue
            sizes[obj["class"]]["miss"].append(height)
            overlapping = [
                f"{d.cls}/{d.source}" for d in result.detections
                if inter_area(obj["box"], (d.x0, d.y0, d.x1, d.y1)) / max(box_area(obj["box"]), 1) > 0.3
            ]
            found_instead[obj["class"]].update(overlapping or ["nothing"])
            if len(crops[obj["class"]]) < 12:
                x0, y0, x1, y1 = [int(v) for v in obj["box"]]
                pad = max(10, (y1 - y0) // 2)
                crop = frame[max(y0 - pad, 0):y1 + pad, max(x0 - pad, 0):x1 + pad]
                crops[obj["class"]].append(cv2.resize(crop, (160, 160)))
        for d in result.detections:
            if not any(obj["class"] == d.cls and is_hit(obj, d) for obj in record["objects"]):
                false_alarms[f"{d.cls}/{d.source}"] += 1

    for cls in sorted(wanted):
        hit, miss = sorted(sizes[cls]["hit"]), sorted(sizes[cls]["miss"])
        median = lambda v: f"{v[len(v) // 2]:.0f} px" if v else "n/a"
        print(f"{cls}: hits {len(hit)} (median height {median(hit)}), misses {len(miss)} (median height {median(miss)})")
        print("   at missed objects:", found_instead[cls].most_common(6))
        if crops[cls]:
            rows = [cv2.hconcat(crops[cls][i:i + 6]) for i in range(0, len(crops[cls]) - len(crops[cls]) % 6, 6)]
            if rows:
                cv2.imwrite(str(args.frames / f"misses_{cls}.png"), cv2.vconcat(rows))
    print("unmatched detections:", false_alarms.most_common(12))


if __name__ == "__main__":
    main()
