#!/usr/bin/env python3
"""Summarize a physical Mixed recording; never declares a physical trial passed."""
import argparse
import collections
import csv
import json
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=pathlib.Path)
    parser.add_argument("--log", type=pathlib.Path)
    parser.add_argument("--geometry-dir", type=pathlib.Path)
    args = parser.parse_args()
    if bool(args.log) != bool(args.geometry_dir):
        parser.error("--log and --geometry-dir must be specified together")
    print(json.dumps(summarize(args.csv), indent=2))
    if args.log:
        export_geometry(args.log, args.geometry_dir)


if __name__ == "__main__":
    main()
