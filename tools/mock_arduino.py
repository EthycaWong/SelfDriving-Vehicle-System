#!/usr/bin/env python3
"""Stands in for the Wokwi board, so the simulation can be tested without VS Code.

Speaks the same lines as firmware/firmware.ino over the same Telnet framing, and checks every
checksum it receives. A script changes the sensors or stops the board at set times:

    mock_arduino.py --script "20:front=30,24:front=400,30:left=50,40:die"

Prints a one-line JSON summary when it exits.
"""
import argparse
import json
import select
import socket
import time

IAC, WILL, SB, SE = 255, 251, 250, 240
COM_PORT_OPTION = 44


def checksum(body):
    value = 0
    for byte in body.encode():
        value ^= byte
    return value


def frame(body):
    return f"${body}*{checksum(body):02X}\r\n".encode()


def parse_script(text):
    events = []
    for item in filter(None, text.split(",")):
        at, action = item.split(":")
        events.append((float(at), action))
    return sorted(events)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=4000)
    parser.add_argument("--script", default="")
    parser.add_argument("--duration", type=float, default=600)
    args = parser.parse_args()

    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", args.port))
    server.listen(1)
    print(f"Mock board listening on {args.port}", flush=True)

    sensors = {"front": 400, "rear": 400, "left": 400, "right": 400}
    events = parse_script(args.script)
    stats = {"commands": 0, "detections": 0, "bad_checksums": 0, "steer_seen": set(), "states": set(), "maneuvers": set()}
    start = time.monotonic()
    server.settimeout(args.duration)
    connection, _ = server.accept()
    connection.setblocking(False)
    # A real RFC2217 server opens with Telnet option negotiation.
    connection.sendall(bytes([IAC, WILL, COM_PORT_OPTION, IAC, SB, COM_PORT_OPTION, 0, IAC, SE]))
    connection.sendall(b"# Mock board ready\r\n")

    buffer = b""
    next_report = time.monotonic()
    alive = True
    start = time.monotonic()
    while alive and time.monotonic() - start < args.duration:
        now = time.monotonic() - start
        while events and events[0][0] <= now:
            _, action = events.pop(0)
            if action == "die":
                alive = False
                break
            name, value = action.split("=")
            sensors[name] = int(value)
        readable, _, _ = select.select([connection], [], [], 0.01)
        if readable:
            try:
                data = connection.recv(4096)
            except OSError:
                break
            if not data:
                break
            buffer += data
            while IAC in buffer:
                index = buffer.index(IAC)
                if len(buffer) < index + 3:
                    break
                buffer = buffer[:index] + buffer[index + 3:]
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if not text.startswith("$") or "*" not in text:
                    continue
                body, _, sum_text = text[1:].rpartition("*")
                if int(sum_text, 16) != checksum(body):
                    stats["bad_checksums"] += 1
                    continue
                fields = body.split(",")
                if fields[0] == "CMD":
                    stats["commands"] += 1
                    stats["states"].add(int(fields[5]))
                    stats["maneuvers"].add(int(fields[6]))
                    stats["steer_seen"].add(int(fields[2]))
                    connection.sendall(frame(f"ACK,{fields[1]}"))
                elif fields[0] == "DET":
                    stats["detections"] += 1
        if time.monotonic() >= next_report:
            next_report += 0.1
            reading = f"SEN,{sensors['front']},{sensors['rear']},{sensors['left']},{sensors['right']},0"
            try:
                connection.sendall(frame(reading))
            except OSError:
                break
    connection.close()
    server.close()
    stats = {key: sorted(value) if isinstance(value, set) else value for key, value in stats.items()}
    print(json.dumps(stats), flush=True)


if __name__ == "__main__":
    main()
