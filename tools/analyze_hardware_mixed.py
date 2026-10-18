#!/usr/bin/env python3
"""Summarize a physical Mixed recording; never declares a physical trial passed."""
import argparse
import collections
import csv
import json
import math
import pathlib


def summarize(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    rows = [row for row in rows if row.get("mixed_state")]
    if not rows:
        raise ValueError("No MixedGoalDirectedDynamic telemetry in this CSV")
    transitions = []
    previous = None
    for row in rows:
        state = row["mixed_state"]
        if state != previous:
            transitions.append({"time_s": float(row["time"]), "state": state,
                                "reason": row["mixed_reason"]})
            previous = state
    last = rows[-1]
    number = lambda name: float(last.get(name) or 0)
    return {
        "recording": str(path), "samples": len(rows),
        "physical_trial_passed": "requires operator review; not inferred",
        "final_state": last["mixed_state"],
        "reported_global_goal_reached": bool(number("mission_goal_reached")),
        "reported_gate_count": number("passed_gates"),
        "rejoins": number("mixed_rejoins"), "replans": number("mixed_replans"),
        "safe_stops": number("mixed_safe_stops"),
        "final_road_s_m": number("mixed_s"), "final_lateral_m": number("mixed_n"),
        "final_heading_error_rad": number("mixed_heading_error"),
        "max_abs_measured_speed_mps": max(abs(float(row["speed"])) for row in rows),
        "max_abs_target_speed_mps": max(abs(float(row["target_speed"])) for row in rows),
        "max_step_ms": max(float(row.get("step_ms") or 0) for row in rows),
        "safe_stop_reasons": dict(collections.Counter(row["mixed_reason"] for row in rows
                                                       if row["mixed_state"] == "SAFE_STOP")),
        "transitions": transitions,
    }


def export_geometry(log_path, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    streams = {}
    try:
        for kind, fields in (("mixed_road", ["x", "y", "yaw", "curvature"]),
                             ("mixed_reference", ["plan", "x", "y", "yaw"])):
            stream = (output_dir / (kind + ".csv")).open("x", newline="")
            writer = csv.DictWriter(stream, fields)
            writer.writeheader()
            streams[kind] = (stream, writer, fields)
        with log_path.open() as source:
            for line in source:
                parts = line.split()
                if not parts or parts[0] not in streams:
                    continue
                values = dict(part.split("=", 1) for part in parts[1:] if "=" in part)
                _, writer, fields = streams[parts[0]]
                writer.writerow({field: values.get(field, "") for field in fields})
    finally:
        for stream, _, _ in streams.values():
            stream.close()


def summarize_raw_scan(path, sector):
    """Inspect capture exports only; raw angles are not mission/body angles."""
    start, end = sector
    if not all(math.isfinite(v) for v in sector) or not 0 < (end - start) % 360 < 360:
        raise ValueError("Raw sector must have distinct finite angular endpoints")
    width = (end - start) % 360
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"scan_index", "angle_deg", "range_m", "quality", "accepted", "rejection_reason"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("Expected a *_raw_scans.csv export from thesis_lidar_start_matcher")
        rows = list(reader)
    selected = [row for row in rows if (float(row["angle_deg"]) - start) % 360 <= width]
    ranges = [float(row["range_m"]) for row in selected
              if math.isfinite(float(row["range_m"])) and float(row["range_m"]) > 0]
    counts = collections.Counter((row.get("acquisition_index", "0"), row["scan_index"]) for row in selected)
    scans = sorted({(row.get("acquisition_index", "0"), row["scan_index"]) for row in rows})
    # Counts alone hide a persistent hole between populated sector edges.
    # Show the actual angles for every scan, without merging different times.
    angle_rows = collections.defaultdict(list)
    for row in selected:
        angle_rows[(row.get("acquisition_index", "0"), row["scan_index"])].append({
            "angle_deg": float(row["angle_deg"]), "range_m": float(row["range_m"]),
            "quality": int(float(row["quality"])), "capture_filter": row["rejection_reason"],
        })
    return {
        "recording": str(path), "raw_sector_deg": list(sector),
        "angle_convention": "sensor raw degrees; car profile 1.8.0 mirrors these into body angles",
        "exported_points": len(rows), "points_in_sector": len(selected),
        "zero_quality_in_sector": sum(float(row["quality"]) == 0 for row in selected),
        "capture_accepted_in_sector": sum(row["accepted"].lower() in ("1", "true") for row in selected),
        "capture_rejection_reasons": dict(collections.Counter(row["rejection_reason"] for row in selected)),
        "range_min_m": min(ranges) if ranges else None,
        "range_max_m": max(ranges) if ranges else None,
        "per_scan": [{"acquisition": a, "scan": s, "sector_points": counts[a, s],
                      "returns": sorted(angle_rows[a, s], key=lambda p: (p["angle_deg"]-start)%360)}
                     for a, s in scans],
        "limitation": "Capture filters differ from navigation filters; missing exported returns do not distinguish sensor occlusion from acquisition loss. This is not motion authorization.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=pathlib.Path, nargs="?")
    parser.add_argument("--log", type=pathlib.Path)
    parser.add_argument("--geometry-dir", type=pathlib.Path)
    parser.add_argument("--raw-start-scan", type=pathlib.Path)
    parser.add_argument("--raw-sector-deg", nargs=2, type=float, metavar=("START", "END"))
    args = parser.parse_args()
    if args.raw_start_scan:
        if args.csv or args.log or args.geometry_dir or args.raw_sector_deg is None:
            parser.error("--raw-start-scan requires --raw-sector-deg START END and no telemetry/geometry arguments")
        print(json.dumps(summarize_raw_scan(args.raw_start_scan, args.raw_sector_deg), indent=2))
        return
    if args.csv is None or args.raw_sector_deg is not None:
        parser.error("provide a telemetry CSV, or --raw-start-scan with --raw-sector-deg")
    if bool(args.log) != bool(args.geometry_dir):
        parser.error("--log and --geometry-dir must be specified together")
    print(json.dumps(summarize(args.csv), indent=2))
    if args.log:
        export_geometry(args.log, args.geometry_dir)


if __name__ == "__main__":
    main()
