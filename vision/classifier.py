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
HORIZON_TILES = 4
TILE_HEIGHT_FRACTION = 0.67

ROAD_CHECK_MIN_AREA = 120
ROAD_CHECK_MIN_HEIGHT = 8
ROAD_CHECK_MIN_SURROUND = 0.35
ROAD_CHECK_MAX_ASPECT = 3.5
ROAD_CHECK_COLOUR_SHIFT = 14


@dataclasses.dataclass
class Detection:
    cls: str
    score: float
    x0: float
    y0: float
    x1: float
    y1: float
    source: str  # "detector", "segmenter" or "road_check"


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


def drop_people_inside_vehicles(detections):
    """People seen through a bus or car window aren't separate road hazards."""
    vehicles = [d for d in detections if d.cls == "vehicle"]
    kept = []
    for detection in detections:
        area = max((detection.x1 - detection.x0) * (detection.y1 - detection.y0), 1.0)
        inside = detection.cls == "person" and any(
            max(0.0, min(detection.x1, v.x1) - max(detection.x0, v.x0))
            * max(0.0, min(detection.y1, v.y1) - max(detection.y0, v.y0)) / area > 0.7
            for v in vehicles)
        if not inside:
            kept.append(detection)
    return kept


def merge_overlaps(detections, threshold=0.45):
    """Keeps the highest-scoring box where boxes of the same class overlap."""
    kept = []
    for detection in sorted(detections, key=lambda d: -d.score):
        if all(k.cls != detection.cls or iou(k, detection) < threshold for k in kept):
            kept.append(detection)
    return kept


class Classifier:
    def __init__(self, models_dir, detector_score=0.3, min_region_fraction=0.002):
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

        labels = json.loads((models_dir / "segformer_config.json").read_text())["id2label"]
        self._ade_lookup = np.full(len(labels), UNLABELLED, np.uint8)
        self._road_index = None
        for index, names in labels.items():
            name = names.split(",")[0].split(";")[0].strip()
            if name == "road":
                self._road_index = int(index)
            cls = ADE_TO_CLASS.get(name)
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
        # the horizon, so overlapping tiles along it are also checked at close to full resolution.
        side = int(height * TILE_HEIGHT_FRACTION)
        top = (height - side) // 2
        step = (width - side) // (HORIZON_TILES - 1)
        for tile in range(HORIZON_TILES):
            left = tile * step
            found += self._detect_in(rgb[top:top + side, left:left + side].copy(), left, top)
        # Far objects bunch up around the vanishing point, so it also gets a closer look.
        zoom = height // 2
        left, top = (width - zoom) // 2, (height - zoom) // 2
        found += self._detect_in(rgb[top:top + zoom, left:left + zoom].copy(), left, top)
        return drop_people_inside_vehicles(merge_overlaps(found))

    def _segment(self, bgr):
        height, width = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(bgr, 1 / 255.0, (SEGMENTER_INPUT, SEGMENTER_INPUT), swapRB=True)
        blob = (blob - SEGMENTER_MEAN) / SEGMENTER_STD
        self._segmenter.setInput(blob.astype(np.float32))
        logits = self._segmenter.forward()[0]
        raw_labels = logits.argmax(axis=0).astype(np.uint8)
        labels = self._ade_lookup[raw_labels]
        segmentation = cv2.resize(labels, (width, height), interpolation=cv2.INTER_NEAREST)
        road = cv2.resize((raw_labels == self._road_index).astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST)

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
        return found, segmentation, road

    def _check_road(self, bgr, road, known):
        """Finds anything sitting on the road that doesn't look like road, whatever it is.

        Neither model has labels for many obstacles, such as rocks and barriers. Treating odd
        colours on the road surface as obstructions catches them without naming them.
        """
        if road.sum() < 500:
            return []
        contours, _ = cv2.findContours(road, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hull = np.zeros(road.shape, np.uint8)
        cv2.fillConvexPoly(hull, cv2.convexHull(np.vstack(contours)), 1)

        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.int16)
        a, b = lab[:, :, 1], lab[:, :, 2]
        usual = road.astype(bool)
        road_a, road_b = np.median(a[usual]), np.median(b[usual])
        colour_shift = np.abs(a - road_a) + np.abs(b - road_b)

        # Colour is what separates an obstacle from the road. Shadows and tar seams are only darker,
        # and white paint is only lighter, so neither counts. Yellow paint is removed separately.
        odd = colour_shift > ROAD_CHECK_COLOUR_SHIFT
        odd &= ~((b - road_b > 25) & (a - road_a < 8))
        mask = (odd & hull.astype(bool)).astype(np.uint8)
        for box in known:
            mask[int(box.y0):int(box.y1) + 1, int(box.x0):int(box.x1) + 1] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        found = []
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        height, width = road.shape
        for component in range(1, count):
            x, y, w, h, area = stats[component]
            if area < ROAD_CHECK_MIN_AREA or h < ROAD_CHECK_MIN_HEIGHT:
                continue
            # The strip of sidewalk at the road's edge is wide and flat. Obstacles stand up.
            if w / h > ROAD_CHECK_MAX_ASPECT:
                continue
            pad = max(4, h // 3)
            x0, y0, x1, y1 = max(x - pad, 0), max(y - pad, 0), min(x + w + pad, width), min(y + h + pad, height)
            ring = road[y0:y1, x0:x1].astype(bool).copy()
            ring[y - y0:y - y0 + h, x - x0:x - x0 + w] = False
            ring_size = (y1 - y0) * (x1 - x0) - w * h
            below = road[min(y + h, height - 1):min(y + h + pad, height), x:x + w]
            if ring_size <= 0 or ring.sum() / ring_size < ROAD_CHECK_MIN_SURROUND or below.size == 0 or below.mean() < 0.5:
                continue
            found.append(Detection("obstruction", 0.5, float(x), float(y), float(x + w), float(y + h), "road_check"))
        return found

    def classify(self, bgr):
        start = time.perf_counter()
        detections = self._detect(bgr)
        middle = time.perf_counter()
        regions, segmentation, road = self._segment(bgr)
        road_objects = self._check_road(bgr, road, detections)
        end = time.perf_counter()
        return Result(detections + regions + road_objects, segmentation, (middle - start) * 1000, (end - middle) * 1000)
