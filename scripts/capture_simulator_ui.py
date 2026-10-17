#!/usr/bin/env python3
"""Render Navigation Lab workspaces without a display, SLAM service or robot.

The last views use explicitly synthetic telemetry to exercise the robot panels.
Requires the GUI build. Pillow is optional (BMPs are kept when unavailable).
"""
import argparse
import os
from pathlib import Path
import subprocess


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=root / "build-ninja/simulator/thesis_planner_sim")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--size", default="1500x920")
    parser.add_argument("--scenario", choices=("structured", "mixed", "unstructured"), default="structured")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, SDL_VIDEODRIVER="dummy", SDL_RENDER_DRIVER="software",
               THESIS_UI_CAPTURE_DIR=str(output), THESIS_UI_CAPTURE_SIZE=args.size)
    command = [str(args.binary.resolve()), "--scenario", args.scenario]
    with (output / "capture.log").open("w") as log:
        subprocess.run(command, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=180)
    try:
        from PIL import Image
    except ImportError:
        Image = None
    for index in range(22):
        path = output / f"view_{index}.bmp"
        if not path.is_file():
            raise RuntimeError(f"Missing capture: {path}")
        if Image:
            with Image.open(path) as image:
                image.save(path.with_suffix(".png"))
            path.unlink()
    (output / "README.txt").write_text(
        "Visual smoke capture; no robot connection.\n"
        "0 overview; 1 SLAM comparison; 2 setup; 3 analysis; 4 editor; "
        "5 diagnostics; 6 export; 7-9 robot without telemetry; "
        "10-14 simulation signal tabs; 15-21 robot with synthetic telemetry.\n"
        "The playback shortcut is exercised before capturing the first view.\n"
    )
    print(f"22 views rendered successfully: {output}")


if __name__ == "__main__":
    main()
