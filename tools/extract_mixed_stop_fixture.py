#!/usr/bin/env python3
"""Reconstruct the stopped 20:11 Mixed scene from exported observations only."""
import csv
import hashlib
import json
import math
from pathlib import Path

root = Path(__file__).resolve().parents[1]
stem = "thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261018_201108_709"
source = root / "reports" / (stem + "_lidar_reconstruction.csv")
snapshot = root / "reports" / (stem + ".json")
history = {q["time"]: q for q in json.loads(snapshot.read_text())["history"]}
groups = {}
with source.open() as stream:
    for row in csv.DictReader(stream):
        groups.setdefault(float(row["time"]), []).append(row)
output = root / "simulator/src/tests/integration/data/mixed_20261018_201108_stopped_scans.txt"
last = max(groups)
selected = {t: scan for t, scan in groups.items() if last - t <= .60}
with output.open("w") as stream:
    for time, scan in sorted(selected.items()):
        h = history[time]
        row = scan[0]
        a, r = float(row["angle_world_rad"]), float(row["range_m"])
        ox, oy = float(row["hit_x"]) - r * math.cos(a), float(row["hit_y"]) - r * math.sin(a)
        uncertainty = (abs(h["speed"]) + 2.5 * abs(h["yaw_rate"])) * (
            h["lidar_scan_duration_s"] + max(0, h["lidar_age_s"]))
        stream.write(f"{time:.6f} {h['position_x']:.6f} {h['position_y']:.6f} "
                     f"{h['yaw']:.6f} {h['curvature']:.6f} {h['speed']:.6f} "
                     f"{ox:.9f} {oy:.9f} {uncertainty:.9f} {len(scan)}\n")
        for row in scan:
            stream.write(f"{row['angle_world_rad']} {row['range_m']}\n")
manifest = {
    "sources": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (source, snapshot)},
    "fixture": str(output.relative_to(root)),
    "fixture_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
    "scans": len(selected), "returns": sum(map(len, selected.values())),
    "start_time": min(selected), "end_time": last,
    "limitations": "Reconstructed endpoints, not raw LiDAR packets. Motion inflation uses exported telemetry. "
                   "Gates are re-extracted; this does not replay the original per-cycle gate search log.",
}
destination = root / "documentation/analysis_20261018/mixed_201108"
destination.mkdir(parents=True, exist_ok=True)
(destination / "fixture_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest, indent=2))
