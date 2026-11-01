#!/usr/bin/env python3
"""Classifier process: receives frames from the C++ simulation and replies with detections.

Frames are header b"FRAM" + id, width, height (little-endian uint32) + width*height*3 RGB bytes.
Replies are one line: id|detectorMs|segmenterMs|class,score,x0,y0,x1,y1,source;...
Only frame pixels cross this link, so the classifier can't see the simulation's labels.
"""
import argparse
import pathlib
import socket
import struct
import sys

import cv2
import numpy as np

from classifier import UNLABELLED, Classifier
from mapping import CLASSES

ROOT = pathlib.Path(__file__).resolve().parent.parent
COLORS = {
    "person": (60, 60, 255), "animal": (0, 165, 255), "vehicle": (255, 120, 0),
    "traffic_control": (255, 0, 255), "tree": (0, 200, 0), "building": (200, 200, 0), "obstruction": (0, 255, 255),
}
OVERLAY = np.zeros((256, 3), np.uint8)
for index, name in enumerate(CLASSES):
    OVERLAY[index] = COLORS[name]


def receive_exactly(connection, size):
    data = bytearray(size)
    view = memoryview(data)
    received = 0
    while received < size:
        count = connection.recv_into(view[received:])
        if count == 0:
            return None
        received += count
    return data


def format_reply(frame_id, result):
    items = ";".join(
        f"{d.cls},{d.score:.2f},{d.x0:.1f},{d.y0:.1f},{d.x1:.1f},{d.y1:.1f},{d.source}" for d in result.detections
    )
    return f"{frame_id}|{result.detector_ms:.1f}|{result.segmenter_ms:.1f}|{items}\n".encode()


def show(bgr, result):
    view = bgr.copy()
    mask = result.segmentation != UNLABELLED
    colored = OVERLAY[result.segmentation]
    view[mask] = (view[mask] * 0.72 + colored[mask] * 0.28).astype(np.uint8)
    for d in result.detections:
        color = COLORS[d.cls]
        thickness = 1 if d.source == "segmenter" else 2
        cv2.rectangle(view, (int(d.x0), int(d.y0)), (int(d.x1), int(d.y1)), color, thickness)
        cv2.putText(view, f"{d.cls} {d.score:.2f}", (int(d.x0) + 2, max(int(d.y0) - 4, 12)), 0, 0.45, color, 1, cv2.LINE_AA)
    cv2.putText(view, f"detector {result.detector_ms:.0f} ms   segmenter {result.segmenter_ms:.0f} ms",
                (10, 22), 0, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow("Classifier view", view)
    cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--models", type=pathlib.Path, default=ROOT / "models")
    parser.add_argument("--no-window", action="store_true")
    args = parser.parse_args()

    classifier = Classifier(args.models)
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", args.port))
    server.listen(1)
    print(f"Classifier ready on port {args.port}", flush=True)

    while True:
        connection, _ = server.accept()
        print("Simulation connected", flush=True)
        with connection:
            while True:
                header = receive_exactly(connection, 16)
                if header is None or header[:4] != b"FRAM":
                    break
                frame_id, width, height = struct.unpack("<III", header[4:])
                pixels = receive_exactly(connection, width * height * 3)
                if pixels is None:
                    break
                rgb = np.frombuffer(pixels, np.uint8).reshape(height, width, 3)
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                result = classifier.classify(bgr)
                connection.sendall(format_reply(frame_id, result))
                if not args.no_window:
                    show(bgr, result)
        print("Simulation disconnected", flush=True)


if __name__ == "__main__":
    sys.exit(main())
