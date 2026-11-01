#!/usr/bin/env python3
"""Scores the classifier against the ground truth written by capture_frames.

The rules were fixed before the first run, and the classifier only ever receives the image:

Which objects count (within 30 m):
  - person, animal, vehicle, traffic_control, obstruction: wholly in frame, with at least 60% of
    the projected box visible and at least 100 visible pixels.
  - tree, building: at least 1500 visible pixels. These are often partly out of frame or hidden.

What counts as a hit (a detection of the same class):
  - Box overlap (IoU) of at least 0.3 with the object's visible box; or
  - for segmentation regions only, covering at least 50% of the visible box, because one region
    can span several neighbouring trees or buildings.

The target is at least 70% recall per class. Precision is reported but isn't part of the target.
"""
import argparse
import json
import pathlib
import sys

import cv2

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "vision"))

from classifier import Classifier  # noqa: E402
from mapping import CLASSES  # noqa: E402

MAX_DISTANCE = 30.0
TARGET_RECALL = 0.70
REGION_CLASSES = {"tree", "building"}
COLORS = {
    "person": (60, 60, 255), "animal": (0, 165, 255), "vehicle": (255, 120, 0),
    "traffic_control": (255, 0, 255), "tree": (0, 200, 0), "building": (200, 200, 0), "obstruction": (0, 255, 255),
}


def box_area(box):
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def inter_area(a, b):
    return box_area((max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])))


def counts(obj):
    if obj["distance"] > MAX_DISTANCE:
        return False
    if obj["class"] in REGION_CLASSES:
        return obj["visible_px"] >= 1500
    projected = box_area(obj["projected"])
    return (obj["in_frame"] and obj["visible_px"] >= 100
            and projected > 0 and box_area(obj["box"]) / projected >= 0.6)


def is_hit(obj, detection):
    if detection.cls != obj["class"]:
        return False
    det = (detection.x0, detection.y0, detection.x1, detection.y1)
    inter = inter_area(obj["box"], det)
    union = box_area(obj["box"]) + box_area(det) - inter
    if union > 0 and inter / union >= 0.3:
        return True
    return detection.source == "segmenter" and inter / max(box_area(obj["box"]), 1.0) >= 0.5


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("frames", type=pathlib.Path)
    parser.add_argument("--models", type=pathlib.Path, default=ROOT / "models")
    parser.add_argument("--annotate-every", type=int, default=25)
    args = parser.parse_args()

    classifier = Classifier(args.models)
    totals = {c: {"objects": 0, "hits": 0, "detections": 0, "matched": 0} for c in CLASSES}
    by_distance = {c: {} for c in CLASSES}
    detector_ms, segmenter_ms = [], []
    annotated = args.frames / "annotated"
    annotated.mkdir(exist_ok=True)

    for number, line in enumerate((args.frames / "ground_truth.jsonl").read_text().splitlines()):
        record = json.loads(line)
        frame = cv2.imread(str(args.frames / record["frame"]))
        result = classifier.classify(frame)
        detector_ms.append(result.detector_ms)
        segmenter_ms.append(result.segmenter_ms)

        for obj in record["objects"]:
            if not counts(obj):
                continue
            hit = any(is_hit(obj, d) for d in result.detections)
            totals[obj["class"]]["objects"] += 1
            totals[obj["class"]]["hits"] += hit
            band = min(int(obj["distance"] // 10) * 10, 20)
            cell = by_distance[obj["class"]].setdefault(band, [0, 0])
            cell[0] += 1
            cell[1] += hit

        # A detection is matched if it overlaps any object of its class, at any distance.
        for detection in result.detections:
            totals[detection.cls]["detections"] += 1
            totals[detection.cls]["matched"] += any(
                obj["class"] == detection.cls and is_hit(obj, detection) for obj in record["objects"])

        if number % args.annotate_every == 0:
            view = frame.copy()
            for d in result.detections:
                color = COLORS[d.cls]
                cv2.rectangle(view, (int(d.x0), int(d.y0)), (int(d.x1), int(d.y1)), color, 1 if d.source == "segmenter" else 2)
                cv2.putText(view, f"{d.cls} {d.score:.2f}", (int(d.x0) + 2, max(int(d.y0) + 12, 12)), 0, 0.45, color, 1, cv2.LINE_AA)
            cv2.imwrite(str(annotated / record["frame"]), view)

    report = {"target_recall": TARGET_RECALL, "max_distance_m": MAX_DISTANCE, "classes": {}}
    print(f"{'class':<17}{'objects':>8}{'hits':>7}{'recall':>9}{'precision':>11}   recall by distance (0-10/10-20/20-30 m)")
    passed = True
    for cls in CLASSES:
        t = totals[cls]
        recall = t["hits"] / t["objects"] if t["objects"] else None
        precision = t["matched"] / t["detections"] if t["detections"] else None
        bands = " / ".join(
            f"{by_distance[cls][b][1] / by_distance[cls][b][0]:.0%} ({by_distance[cls][b][0]})" if b in by_distance[cls] else "-"
            for b in (0, 10, 20))
        recall_text = f"{recall:.0%}" if recall is not None else "n/a"
        precision_text = f"{precision:.0%}" if precision is not None else "n/a"
        print(f"{cls:<17}{t['objects']:>8}{t['hits']:>7}{recall_text:>9}{precision_text:>11}   {bands}")
        passed &= recall is not None and recall >= TARGET_RECALL
        report["classes"][cls] = {**t, "recall": recall, "precision": precision}

    detector_ms.sort()
    segmenter_ms.sort()
    report["detector_ms_median"] = detector_ms[len(detector_ms) // 2]
    report["segmenter_ms_median"] = segmenter_ms[len(segmenter_ms) // 2]
    report["passed"] = passed
    print(f"median time: detector {report['detector_ms_median']:.0f} ms, segmenter {report['segmenter_ms_median']:.0f} ms")
    print("TARGET MET" if passed else "TARGET NOT MET")
    (args.frames / "scorecard.json").write_text(json.dumps(report, indent=1))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
