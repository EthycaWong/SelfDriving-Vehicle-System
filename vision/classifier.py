"""Classifies what's in a camera frame, using only the frame's pixels.

MediaPipe's object detector finds countable things such as people, animals and vehicles. Its COCO
labels have no tree or building, so a SegFormer model run through cv2.dnn labels those regions, and
cv2 turns each region into a box.
"""
import dataclasses
import json
import pathlib
import time

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from mapping import ADE_TO_CLASS, CLASSES, COCO_TO_CLASS

SEGMENTER_INPUT = 512
SEGMENTER_MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
SEGMENTER_STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)
UNLABELLED = 255


@dataclasses.dataclass
class Detection:
    cls: str
    score: float
    x0: float
    y0: float
    x1: float
    y1: float
    source: str  # "detector" or "segmenter"


@dataclasses.dataclass
class Result:
    detections: list
    segmentation: np.ndarray  # Frame-sized map of class indices; UNLABELLED elsewhere.
    detector_ms: float
    segmenter_ms: float


def iou(a, b):
    ix = max(0.0, min(a.x1, b.x1) - max(a.x0, b.x0))
    iy = max(0.0, min(a.y1, b.y1) - max(a.y0, b.y0))
    inter = ix * iy
    union = (a.x1 - a.x0) * (a.y1 - a.y0) + (b.x1 - b.x0) * (b.y1 - b.y0) - inter
    return inter / union if union > 0 else 0.0


def merge_overlaps(detections, threshold=0.45):
    """Keeps the highest-scoring box where boxes of the same class overlap."""
    kept = []
    for detection in sorted(detections, key=lambda d: -d.score):
        if all(k.cls != detection.cls or iou(k, detection) < threshold for k in kept):
            kept.append(detection)
    return kept


class Classifier:
    def __init__(self, models_dir, detector_score=0.3, min_region_fraction=0.002, horizon_crop=0.5):
        models_dir = pathlib.Path(models_dir)
        options = mp_vision.ObjectDetectorOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(models_dir / "efficientdet_lite0.tflite")),
            running_mode=mp_vision.RunningMode.IMAGE,
            score_threshold=detector_score,
            max_results=40,
        )
        self._detector = mp_vision.ObjectDetector.create_from_options(options)
        self._segmenter = cv2.dnn.readNetFromONNX(str(models_dir / "segformer_b0_ade.onnx"))
        self._min_region_fraction = min_region_fraction
        self._horizon_crop = horizon_crop

        labels = json.loads((models_dir / "segformer_config.json").read_text())["id2label"]
        self._ade_lookup = np.full(len(labels), UNLABELLED, np.uint8)
        for index, names in labels.items():
            cls = ADE_TO_CLASS.get(names.split(",")[0].split(";")[0].strip())
            if cls is not None:
                self._ade_lookup[int(index)] = CLASSES.index(cls)

    def _detect_in(self, rgb, offset_x, offset_y):
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        found = []
        for item in self._detector.detect(image).detections:
            category = item.categories[0]
            cls = COCO_TO_CLASS.get(category.category_name)
            if cls is None:
                continue
            box = item.bounding_box
            found.append(Detection(
                cls, category.score,
                box.origin_x + offset_x, box.origin_y + offset_y,
                box.origin_x + box.width + offset_x, box.origin_y + box.height + offset_y,
                "detector",
            ))
        return found

    def _detect(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        found = self._detect_in(rgb, 0, 0)
        # The detector shrinks its input to 320x320, which loses distant objects. Those sit near
        # the horizon, so a second pass looks at that area at a higher zoom.
        crop_w, crop_h = int(width * self._horizon_crop), int(height * self._horizon_crop)
        x0, y0 = (width - crop_w) // 2, (height - crop_h) // 2
        found += self._detect_in(rgb[y0:y0 + crop_h, x0:x0 + crop_w].copy(), x0, y0)
        return merge_overlaps(found)

    def _segment(self, bgr):
        height, width = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(bgr, 1 / 255.0, (SEGMENTER_INPUT, SEGMENTER_INPUT), swapRB=True)
        blob = (blob - SEGMENTER_MEAN) / SEGMENTER_STD
        self._segmenter.setInput(blob.astype(np.float32))
        logits = self._segmenter.forward()[0]
        labels = self._ade_lookup[logits.argmax(axis=0)]
        segmentation = cv2.resize(labels, (width, height), interpolation=cv2.INTER_NEAREST)

        min_area = self._min_region_fraction * width * height
        kernel = np.ones((5, 5), np.uint8)
        found = []
        for index in np.unique(labels):
            if index == UNLABELLED:
                continue
            mask = cv2.morphologyEx((segmentation == index).astype(np.uint8), cv2.MORPH_CLOSE, kernel)
            count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for component in range(1, count):
                x, y, w, h, area = stats[component]
                if area < min_area:
                    continue
                found.append(Detection(CLASSES[index], 1.0, float(x), float(y), float(x + w), float(y + h), "segmenter"))
        return found, segmentation

    def classify(self, bgr):
        start = time.perf_counter()
        detections = self._detect(bgr)
        middle = time.perf_counter()
        regions, segmentation = self._segment(bgr)
        end = time.perf_counter()
        return Result(detections + regions, segmentation, (middle - start) * 1000, (end - middle) * 1000)
