# SelfDrive

A self-driving demo with three parts that talk to each other:

- **Simulation (C++ and raylib):** renders a 3D road scene and drives a car through it.
- **Classifier (Python, MediaPipe and cv2):** labels people, animals, vehicles, traffic signs, trees, buildings and obstructions, using only the rendered pixels.
- **Arduino (Wokwi):** a simulated Uno that receives commands and sends sensor data back.

## Status

Work in progress. The renderer, asset tools and classifier exist. The recognition test, the Wokwi link and the driving modules are still to come.

## Third-party assets

Models and scene assets aren't stored in this repo. `tools/fetch_assets.py` downloads the scene assets.

- **Poly Haven models and textures:** CC0.
- **COCO photos:** only images licensed as attribution, no known copyright or US Government work. The script records attributions in `assets/cutouts/ATTRIBUTION.json`.
- **SegFormer-B0:** the upstream NVIDIA licence is listed as "other". Check it before any commercial use.
