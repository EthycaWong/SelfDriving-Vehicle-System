# SelfDrive

A self-driving demo where you can watch every module work. A C++ program renders a 3D city and drives a car through it, a Python classifier labels what the car's camera sees, and a simulated ESP32-S2 in Wokwi exchanges commands and sensor readings with the car.

```
 C++ simulation (raylib)                 Python classifier
 ┌──────────────────────────┐  frames    ┌───────────────────────┐
 │ World and Renderer ──────┼───────────►│ MediaPipe detector    │
 │ Classifier link ◄────────┼────────────┤ SegFormer via cv2     │
 │      │  message bus      │ detections │ cv2 road check        │
 │ Perception → Planner     │            └───────────────────────┘
 │  → Maneuver → Controller │  serial over TCP 4000
 │  → Vehicle               │◄──────────► Wokwi ESP32-S2 with four
 │ Board link, Dashboard    │             HC-SR04 ultrasonic sensors
 └──────────────────────────┘
```

The classifier only ever receives pixels. The simulation knows what it placed, but that's only used to score the classifier on the dashboard. It never reaches the Planner, so the car only reacts to what the classifier actually saw.

## Set up

```bash
tools/setup.sh
```

For the Wokwi board, also install [Wokwi for VS Code](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode), press `F1` and choose **Wokwi: Request a new License**.

## Run

With the real Wokwi board:

1. Open this folder in VS Code.
2. Press `F1` and choose **Wokwi: Start Simulator**. Keep its tab visible, because the simulation may pause otherwise.
3. Start everything else:

```bash
tools/run_all.sh
```

Without Wokwi, using a stand-in board:

```bash
tools/run_all.sh --mock
```

## Controls

| Key | Action |
|---|---|
| `M` | Manual driving: `WASD` or arrow keys, `R` for reverse gear |
| `1` to `8` | Lane change, overtake, go around, swerve, turn, U-turn, reverse, pull over |
| `9` | Rejoin the lane after pulling over |
| `P` `N` `V` `O` | Drop a person, animal, vehicle or obstacle ahead of the car |
| `C` | Remove what you dropped |

In Wokwi, click a sensor and drag its distance slider:

| Sensor | Effect when something is close |
|---|---|
| Front | Below 60 cm: emergency stop |
| Rear | Below 100 cm: blocks reversing |
| Left and right | Below 100 cm: blocks moves toward that side |

The sliders are separate from the 3D world, so they don't change when the car passes something. The board's messages appear in the Wokwi serial monitor.

## What the classifier can do

Scored on a scene it wasn't tuned on, for objects within 30 m. The target was 70% recall per class. See [the scorecard](docs/recognition_scorecard.txt) and the rules in `tools/score_recognition.py`.

| Class | Recall | Precision |
|---|---|---|
| Person | 100% | 49% |
| Vehicle | 100% | 88% |
| Building | 96% | 81% |
| Tree | 89% | 81% |
| Traffic control | 62% (below target) | 66% |
| Obstruction | 60% (below target) | 47% |
| Animal | 59% (below target) | 91% |

Known limits:

- About half of animals, signs and obstructions are missed at 20-30 m.
- Person precision is low, so the car sometimes brakes for something that isn't there.
- Scores come from this rendered scene and don't predict real-world performance.
- At the 8 m/s demo speed, braking takes less distance than swerving. An automatic swerve mainly helps at higher speeds.

## Tests

```bash
ctest --test-dir build
```

```bash
.venv/bin/python tools/e2e_test.py
```

The first runs the protocol tests and a scripted scene for each maneuver. The second runs the real simulation and classifier together with the stand-in board.

## Third-party assets

Models and scene assets aren't stored in this repo. `tools/fetch_assets.py` downloads them.

- **Poly Haven models and textures:** CC0.
- **COCO photos:** only images licensed as attribution, no known copyright or US Government work. Attributions are recorded in `assets/cutouts/ATTRIBUTION.json`.
- **SegFormer-B0:** the upstream NVIDIA licence is listed as "other". Check it before any commercial use.
