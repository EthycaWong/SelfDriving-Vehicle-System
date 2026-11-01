#!/usr/bin/env python3
"""Downloads scene assets: COCO photo cutouts and Poly Haven (CC0) models and textures.

COCO images are filtered by licence and by cutout geometry only. They are never
filtered by whether the detector recognises them, because that would inflate the
recognition score.
"""
import argparse
import json
import pathlib
import ssl
import urllib.request
import zipfile

import certifi
import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
ASSETS = ROOT / "assets"
RAW = ASSETS / "raw"
USER_AGENT = {"User-Agent": "SelfDrive-demo/1.0"}
TLS = ssl.create_default_context(cafile=certifi.where())

COCO_ANNOTATIONS = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"
COCO_MEMBER = "annotations/instances_val2017.json"
ALLOWED_LICENSE_NAMES = (
    "Attribution License",
    "No known copyright restrictions",
    "United States Government Work",
)

# Scene class, COCO category, cutouts to keep, allowed height/width range, real height in metres,
# smallest usable side in source pixels, height above ground (signs sit on a pole).
CUTOUTS = [
    ("person", "person", 9, (1.8, 4.5), 1.75, 60, 0.0),
    ("animal", "dog", 3, (0.5, 1.6), 0.7, 40, 0.0),
    ("animal", "horse", 3, (0.6, 1.4), 1.6, 40, 0.0),
    ("animal", "cow", 3, (0.5, 1.2), 1.5, 40, 0.0),
    ("animal", "sheep", 2, (0.5, 1.2), 0.9, 40, 0.0),
    ("vehicle", "car", 6, (0.35, 1.1), 1.5, 60, 0.0),
    ("vehicle", "truck", 3, (0.4, 1.2), 3.0, 60, 0.0),
    ("vehicle", "bus", 3, (0.4, 1.2), 3.2, 60, 0.0),
    ("traffic_control", "stop sign", 3, (0.8, 1.3), 0.9, 60, 2.0),
    ("traffic_control", "traffic light", 3, (1.3, 4.0), 1.0, 18, 3.0),
]

# Removed after looking at the contact sheet, for bad masks only. Never removed because of how
# the detector scores them.
REJECTED_CUTOUTS = {
    "person_2.png",  # Mask stops at the knee, so the figure would float above the road.
}

POLYHAVEN_MODELS = [
    "tree_small_02",
    "jacaranda_tree",
    "island_tree_01",
    "concrete_road_barrier",
    "boulder_01",
    "Barrel_01",
    "fire_hydrant",
    "painted_wooden_bench",
    "street_lamp_01",
]
POLYHAVEN_TEXTURES = [
    "asphalt_02",
    "concrete_pavement",
    "sparse_grass",
    "brick_wall_02",
    "brick_wall_09",
    "sandstone_brick_wall_01",
    "concrete_tile_facade",
]
POLYHAVEN_SKY = "citrus_orchard_road_puresky"


def download(url, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        return path
    print(f"  downloading {url}")
    request = urllib.request.Request(url, headers=USER_AGENT)
    with urllib.request.urlopen(request, timeout=300, context=TLS) as response:
        data = response.read()
    path.write_bytes(data)
    return path


def fetch_json(url):
    request = urllib.request.Request(url, headers=USER_AGENT)
    with urllib.request.urlopen(request, timeout=60, context=TLS) as response:
        return json.load(response)


def load_coco():
    extracted = RAW / "instances_val2017.json"
    if not extracted.exists():
        archive = download(COCO_ANNOTATIONS, RAW / "annotations_trainval2017.zip")
        with zipfile.ZipFile(archive) as zf, zf.open(COCO_MEMBER) as src:
            extracted.write_bytes(src.read())
        archive.unlink()
    return json.loads(extracted.read_text())


def overlap_fraction(a, b):
    """Fraction of box a that box b covers. Boxes are x, y, w, h."""
    ix = max(0.0, min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1]))
    return ix * iy / max(a[2] * a[3], 1.0)


def fetch_cutouts(coco):
    allowed = {l["id"] for l in coco["licenses"] if l["name"] in ALLOWED_LICENSE_NAMES}
    images = {i["id"]: i for i in coco["images"] if i["license"] in allowed}
    licences = {l["id"]: l for l in coco["licenses"]}
    categories = {c["name"]: c["id"] for c in coco["categories"]}
    per_image = {}
    for ann in coco["annotations"]:
        per_image.setdefault(ann["image_id"], []).append(ann)

    rows, attributions = [], []
    for scene_class, category, count, (min_ratio, max_ratio), height_m, min_side, mount_m in CUTOUTS:
        candidates = []
        for ann in coco["annotations"]:
            if ann["category_id"] != categories[category] or ann["iscrowd"]:
                continue
            image = images.get(ann["image_id"])
            if image is None or not isinstance(ann["segmentation"], list):
                continue
            x, y, w, h = ann["bbox"]
            # Whole object in view, big enough to stay sharp, solid mask, not overlapped.
            if x < 3 or y < 3 or x + w > image["width"] - 3 or y + h > image["height"] - 3:
                continue
            if min(w, h) < min_side or not min_ratio <= h / w <= max_ratio:
                continue
            if ann["area"] / (w * h) < 0.4 or len(ann["segmentation"]) > 2:
                continue
            others = [o for o in per_image[ann["image_id"]] if o["id"] != ann["id"]]
            if any(overlap_fraction(ann["bbox"], o["bbox"]) > 0.03 for o in others):
                continue
            candidates.append(ann)
        candidates.sort(key=lambda a: -a["area"])

        kept = 0
        used_images = set()
        for ann in candidates:
            if kept == count:
                break
            if ann["image_id"] in used_images:
                continue
            used_images.add(ann["image_id"])
            image = images[ann["image_id"]]
            path = download(image["coco_url"], RAW / "coco" / image["file_name"])
            photo = cv2.imread(str(path))
            if photo is None:
                continue
            mask = np.zeros(photo.shape[:2], np.uint8)
            for polygon in ann["segmentation"]:
                pts = np.array(polygon, np.float32).reshape(-1, 2)
                cv2.fillPoly(mask, [np.round(pts).astype(np.int32)], 255, lineType=cv2.LINE_AA)
            x, y, w, h = [int(round(v)) for v in ann["bbox"]]
            pad = 2
            x0, y0 = max(x - pad, 0), max(y - pad, 0)
            x1, y1 = min(x + w + pad, photo.shape[1]), min(y + h + pad, photo.shape[0])
            rgba = cv2.cvtColor(photo[y0:y1, x0:x1], cv2.COLOR_BGR2BGRA)
            rgba[:, :, 3] = cv2.GaussianBlur(mask[y0:y1, x0:x1], (3, 3), 0)
            name = f"{category.replace(' ', '_')}_{kept}.png"
            kept += 1
            if name in REJECTED_CUTOUTS:
                continue
            out = ASSETS / "cutouts" / scene_class / name
            out.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out), rgba)
            rows.append("\t".join([
                str(out.relative_to(ROOT)), scene_class, category,
                f"{height_m}", f"{(x1 - x0) / (y1 - y0):.4f}", f"{mount_m}",
            ]))
            attributions.append({
                "file": str(out.relative_to(ROOT)),
                "source": image.get("flickr_url", image["coco_url"]),
                "licence": licences[image["license"]]["name"],
                "licence_url": licences[image["license"]]["url"],
            })
        print(f"  {category}: kept {kept} of {len(candidates)} candidates")

    (ASSETS / "cutouts" / "manifest.tsv").write_text("\n".join(rows) + "\n")
    (ASSETS / "cutouts" / "ATTRIBUTION.json").write_text(json.dumps(attributions, indent=1))


def fetch_model(name):
    files = fetch_json(f"https://api.polyhaven.com/files/{name}")
    entry = files["gltf"]["1k"]["gltf"]
    folder = ASSETS / "models" / name
    download(entry["url"], folder / f"{name}.gltf")
    total = entry["size"]
    for relative, item in entry.get("include", {}).items():
        download(item["url"], folder / relative)
        total += item["size"]
    print(f"  model {name}: {total / 1e6:.1f} MB")


def fetch_texture(name):
    files = fetch_json(f"https://api.polyhaven.com/files/{name}")
    download(files["Diffuse"]["1k"]["jpg"]["url"], ASSETS / "textures" / f"{name}.jpg")


def convert_jpegs_to_png():
    """Homebrew's raylib is built without JPG support, so textures and model references become PNG."""
    for jpeg in list((ASSETS / "textures").glob("*.jpg")) + list((ASSETS / "models").glob("*/textures/*.jpg")):
        png = jpeg.with_suffix(".png")
        if not png.exists():
            cv2.imwrite(str(png), cv2.imread(str(jpeg)))
    for gltf in (ASSETS / "models").glob("*/*.gltf"):
        text = gltf.read_text()
        if ".jpg" in text:
            gltf.write_text(text.replace(".jpg", ".png"))


def make_facades():
    """Builds one-storey wall tiles with windows from the brick textures. A tile is 7 m by 3.5 m."""
    sources = ["brick_wall_02", "brick_wall_09", "sandstone_brick_wall_01", "concrete_tile_facade"]
    for number, name in enumerate(sources):
        wall = cv2.imread(str(ASSETS / "textures" / f"{name}.jpg"))
        wall = cv2.resize(wall, (1024, 512), interpolation=cv2.INTER_AREA)
        for bay in range(2):
            x0, x1 = 90 + bay * 512, 422 + bay * 512
            y0, y1 = 110, 400
            cv2.rectangle(wall, (x0 - 14, y0 - 14), (x1 + 14, y1 + 14), (205, 205, 200), -1)
            cv2.rectangle(wall, (x0 - 22, y1 + 14), (x1 + 22, y1 + 28), (170, 170, 165), -1)
            # Glass reflects a little sky: lighter at the top and darker toward the bottom.
            ramp = np.linspace(1.0, 0.0, y1 - y0, dtype=np.float32).reshape(-1, 1, 1)
            top, bottom = np.array((150, 125, 95), np.float32), np.array((62, 52, 45), np.float32)
            wall[y0:y1, x0:x1] = (ramp * top + (1 - ramp) * bottom).astype(np.uint8)
            middle = (x0 + x1) // 2
            cv2.rectangle(wall, (middle - 6, y0), (middle + 6, y1), (205, 205, 200), -1)
            cv2.rectangle(wall, (x0, 210), (x1, 222), (205, 205, 200), -1)
        cv2.imwrite(str(ASSETS / "textures" / f"facade_{number}.png"), wall)


def fetch_vision_models():
    models = ROOT / "models"
    download(
        "https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/float16/latest/efficientdet_lite0.tflite",
        models / "efficientdet_lite0.tflite",
    )
    segformer = "https://huggingface.co/Xenova/segformer-b0-finetuned-ade-512-512"
    download(f"{segformer}/resolve/main/onnx/model.onnx", models / "segformer_b0_ade.onnx")
    download(f"{segformer}/raw/main/config.json", models / "segformer_config.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-coco", action="store_true")
    parser.add_argument("--skip-polyhaven", action="store_true")
    args = parser.parse_args()

    fetch_vision_models()
    if not args.skip_polyhaven:
        print("Poly Haven:")
        for name in POLYHAVEN_MODELS:
            fetch_model(name)
        for name in POLYHAVEN_TEXTURES:
            fetch_texture(name)
        sky = fetch_json(f"https://api.polyhaven.com/files/{POLYHAVEN_SKY}")
        download(sky["tonemapped"]["url"], ASSETS / "textures" / "sky.jpg")
        make_facades()
        convert_jpegs_to_png()
    if not args.skip_coco:
        print("COCO:")
        fetch_cutouts(load_coco())


if __name__ == "__main__":
    main()
