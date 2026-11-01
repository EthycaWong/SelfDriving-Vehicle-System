#!/usr/bin/env python3
"""Runs the real simulation and the real classifier together, with the mock board.

Checks that messages flow everywhere, and that the car reacts to a person in its lane, to the
board's front sensor, to losing the board and to losing the classifier.
"""
import json
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
PYTHON = str(ROOT / ".venv" / "bin" / "python")
SIM = str(ROOT / "build" / "selfdrive_sim")
VISION_PORT, BOARD_PORT = "15555", "14000"


def start_vision():
    process = subprocess.Popen(
        [PYTHON, str(ROOT / "vision" / "vision_server.py"), "--no-window", "--port", VISION_PORT],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, cwd=ROOT / "vision")
    for line in process.stdout:
        if "ready" in line:
            return process
    raise SystemExit("Classifier didn't start")


def records(path):
    rows = [json.loads(line) for line in pathlib.Path(path).read_text().splitlines()]
    return [r for r in rows if "t" in r], next((r for r in rows if r.get("final")), {})


def check(name, passed, detail=""):
    print(f"  {'PASS' if passed else 'FAIL'}  {name}{'  ' + str(detail) if detail else ''}")
    return passed


def run_sim(log, duration, events, seed, shots=None):
    command = [SIM, "--root", str(ROOT), "--hidden", "--duration", str(duration), "--log", log,
               "--vision-port", VISION_PORT, "--board-port", BOARD_PORT, "--seed", str(seed), "--events", events]
    if shots:
        command += ["--shots", shots]
    return subprocess.Popen(command, stderr=subprocess.DEVNULL)


def scenario_person_and_sensors(folder):
    print("Person in lane, front sensor, then losing the board:")
    log = f"{folder}/a.jsonl"
    board = subprocess.Popen(
        [PYTHON, str(ROOT / "tools" / "mock_arduino.py"), "--port", BOARD_PORT, "--script", "32:front=30,37:front=400,50:die"],
        stdout=subprocess.PIPE, text=True)
    time.sleep(0.5)
    sim = run_sim(log, 64, "16:clear,16.1:person,17:shot,27:clear,34:shot,44:shot,60:shot", 11, f"{ROOT}/spikes/out/e2e_")
    sim.wait()
    board_output = board.communicate(timeout=10)[0].strip().splitlines()[-1]
    stats = json.loads(board_output)
    rows, final = records(log)

    ok = check("classifier replied with detections", max(r["frames"] for r in rows) > 50 and any(r["detections"] > 0 for r in rows))
    ok &= check("board received commands with good checksums", stats["commands"] > 500 and stats["bad_checksums"] == 0,
                f"{stats['commands']} commands, {stats['bad_checksums']} bad")
    ok &= check("car drove", max(abs(r["speed"]) for r in rows) > 5)
    after_person = [r for r in rows if 16 < r["t"] < 27]
    ok &= check("stopped or slowed for the person", any(r["state"] == "STOP" for r in after_person),
                sorted({r["state"] for r in after_person}))
    estop = [r for r in rows if 33 < r["t"] < 36.5]
    ok &= check("front sensor caused ESTOP", all(r["state"] == "ESTOP" for r in estop) and abs(estop[-1]["speed"]) < 0.1)
    ok &= check("car moved again after ESTOP", any(abs(r["speed"]) > 1 for r in rows if 38 < r["t"] < 50))
    lost = [r for r in rows if r["t"] > 53]
    ok &= check("losing the board caused FAILSAFE", all(r["state"] == "FAILSAFE" for r in lost))
    ok &= check("pulled over and stopped", abs(lost[-1]["speed"]) < 0.1 and lost[-1]["maneuver"] == "PULL OVER",
                f"speed {lost[-1]['speed']}, {lost[-1]['maneuver']}")
    print("  collisions:", final.get("collisions"), " live score (hits, objects):", final.get("scores"))
    return ok


def scenario_vision_loss(folder, vision):
    print("Losing the classifier, then getting it back:")
    log = f"{folder}/b.jsonl"
    board = subprocess.Popen(
        [PYTHON, str(ROOT / "tools" / "mock_arduino.py"), "--port", BOARD_PORT, "--duration", "60"],
        stdout=subprocess.DEVNULL)
    time.sleep(0.5)
    sim = run_sim(log, 50, "1:shot", 12)
    time.sleep(16)
    vision.send_signal(signal.SIGTERM)
    vision.wait()
    time.sleep(14)
    vision = start_vision()
    sim.wait()
    board.terminate()
    rows, _ = records(log)
    ok = check("FAILSAFE while the classifier was gone", any(r["state"] == "FAILSAFE" and not r["vision"] for r in rows))
    tail = rows[-8:]
    ok &= check("recovered after it restarted", all(r["vision"] and r["state"] != "FAILSAFE" for r in tail), tail[-1]["state"])
    return ok, vision


def main():
    vision = start_vision()
    try:
        with tempfile.TemporaryDirectory() as folder:
            passed = scenario_person_and_sensors(folder)
            result, vision = scenario_vision_loss(folder, vision)
            passed &= result
    finally:
        vision.terminate()
    print("END TO END:", "PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
