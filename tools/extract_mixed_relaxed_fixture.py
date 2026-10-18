#!/usr/bin/env python3
"""Extract observed endpoints only; no invented rays, obstacles, or gate widths."""
import csv
import hashlib
import json
import math
from pathlib import Path

root = Path(__file__).resolve().parents[1]
name = 'thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261018_101839_285'
source = root / 'reports' / (name + '_lidar_reconstruction.csv')
rows = list(csv.DictReader(source.open()))
last = max(float(r['time']) for r in rows)
groups = {}
for r in rows:
    t = float(r['time'])
    if last - t <= .60:
        groups.setdefault(t, []).append(r)
output = root / 'simulator/src/tests/integration/data/mixed_20261018_101839_observed_scans.txt'
with output.open('w') as f:
    for time, scan in sorted(groups.items()):
        r = scan[0]
        x, y, yaw = (float(r[k]) for k in ('pose_x', 'pose_y', 'pose_yaw'))
        # Origin is exported indirectly by endpoint/range/angle; use the actual
        # reconstruction, not an assumed sensor mounting or a new transform.
        a, distance = float(r['angle_world_rad']), float(r['range_m'])
        ox = float(r['hit_x']) - distance * math.cos(a)
        oy = float(r['hit_y']) - distance * math.sin(a)
        f.write(f'{time:.6f} {x:.6f} {y:.6f} {yaw:.6f} {ox:.9f} {oy:.9f} {len(scan)}\n')
        for r in scan:
            f.write(f"{r['angle_world_rad']} {r['range_m']}\n")
manifest = {
    'source': str(source.relative_to(root)),
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'fixture': str(output.relative_to(root)),
    'fixture_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
    'scans': len(groups), 'returns': sum(map(len, groups.values())),
    'start_time': min(groups), 'end_time': last,
    'limitations': 'Filtered reconstructed endpoints, not original raw packets. Quality, scan duration and exact per-gate search poses/widths are unavailable. Gate extraction is rerun on these same endpoints for both policies. This is not a replay of all 258 original gate searches.'
}
(root / 'documentation/analysis_20261018/experimental_relaxed/fixture_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
print(json.dumps(manifest, indent=2))
