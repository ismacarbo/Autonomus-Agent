#!/usr/bin/env python3
"""Send a pose stream to thesis_robot_runner's reference-only UDP input."""

from __future__ import annotations

import argparse
import math
import socket
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15150)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw-deg", type=float, required=True)
    parser.add_argument("--quality", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--duration-s", type=float, default=2.0)
    args = parser.parse_args()

    period = 1.0 / max(args.rate_hz, 1.0)
    deadline = time.monotonic() + max(args.duration_s, 0.0)
    target = (args.host, args.port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        while time.monotonic() <= deadline:
            payload = (
                f"{time.time():.9f},{args.x:.9f},{args.y:.9f},"
                f"{math.radians(args.yaw_deg):.9f},{args.quality:.6f}"
            )
            sender.sendto(payload.encode("ascii"), target)
            time.sleep(period)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
