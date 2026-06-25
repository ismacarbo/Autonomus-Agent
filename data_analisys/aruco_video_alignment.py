"""Extract ArUco ground-truth poses from a video and align them to a report.

The script assumes the video is a concatenation of valid real-time clips. It
therefore splits visible ArUco poses into continuous segments and aligns each
segment independently against the hardware report timeline.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

from data_analisys import aruco_pose_analysis as aruco


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("data_analisys/aruco_ground_truth_config.json"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("data_analisys/outputs/aruco_video_alignment"))
    parser.add_argument("--robot-id", type=int)
    parser.add_argument("--dictionary", action="append")
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--search-step-s", type=float, default=0.10)
    parser.add_argument("--min-segment-samples", type=int, default=8)
    parser.add_argument("--max-gap-s", type=float, default=0.45)
    parser.add_argument("--max-jump-m", type=float, default=0.10)
    parser.add_argument("--max-yaw-jump-rad", type=float, default=0.90)
    parser.add_argument(
        "--video-filter-start-s",
        type=float,
        default=None,
        help="Discard ArUco detections before this absolute video time without forcing manual log alignment.",
    )
    parser.add_argument(
        "--video-filter-end-s",
        type=float,
        default=None,
        help="Discard ArUco detections after this absolute video time without forcing manual log alignment.",
    )
    parser.add_argument(
        "--alignment-mode",
        choices=("monotonic", "independent"),
        default="monotonic",
        help="monotonic keeps video segment order in the report; independent picks the best window per segment",
    )
    parser.add_argument(
        "--video-log-start-s",
        type=float,
        default=None,
        help="Absolute video time where report time --report-log-start-s starts. Overrides segment search when provided.",
    )
    parser.add_argument(
        "--report-log-start-s",
        type=float,
        default=0.0,
        help="Report time corresponding to --video-log-start-s.",
    )
    parser.add_argument(
        "--robot-to-marker-x-m",
        type=float,
        default=0.0,
        help="Marker center x in the robot frame; +x is robot forward",
    )
    parser.add_argument(
        "--robot-to-marker-y-m",
        type=float,
        default=0.0,
        help="Marker center y in the robot frame; +y is robot right/down according to the map convention",
    )
    parser.add_argument(
        "--marker-to-robot-yaw-offset-deg",
        type=float,
        default=None,
        help="Fixed alpha where yaw_robot = yaw_marker + alpha. If omitted, alpha is fitted per segment.",
    )
    parser.add_argument(
        "--report-registration",
        choices=("none", "similarity", "affine", "initial_rigid"),
        default="none",
        help="Fit/apply a report-to-real-map transform. initial_rigid uses only the first aligned pose.",
    )
    parser.add_argument(
        "--report-registration-file",
        type=Path,
        help="JSON file with a precomputed report-to-real-map registration. Overrides --report-registration.",
    )
    parser.add_argument(
        "--report-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Extra yaw offset applied to the report estimate after report-to-real registration.",
    )
    parser.add_argument(
        "--report-yaw-offset-by-segment",
        help="Comma-separated segment-specific report yaw offsets, e.g. 0:180,2:180. Overrides the global offset for those segments.",
    )
    parser.add_argument("--write-overlay-video", action="store_true")
    parser.add_argument("--write-map-video", action="store_true")
    parser.add_argument("--map-pixels-per-meter", type=float, default=650.0)
    parser.add_argument("--map-padding-px", type=int, default=70)
    return parser.parse_args(argv)


def wrap_angle(angle: float) -> float:
    return aruco.wrap_angle(angle)


def angle_delta(a: float, b: float) -> float:
    return wrap_angle(a - b)


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * pct / 100.0
    lo = int(math.floor(index))
    hi = int(math.ceil(index))
    if lo == hi:
        return ordered[lo]
    alpha = index - lo
    return ordered[lo] * (1.0 - alpha) + ordered[hi] * alpha


def stats(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "mean": statistics.fmean(values) if values else None,
        "median": percentile(values, 50.0),
        "p95": percentile(values, 95.0),
        "max": max(values) if values else None,
    }


def parse_segment_yaw_offsets(text: str | None) -> dict[int, float]:
    if not text:
        return {}
    offsets: dict[int, float] = {}
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise SystemExit(f"Offset segmento non valido, atteso SEGMENTO:GRADI: {item}")
        segment_text, offset_text = item.split(":", 1)
        offsets[int(segment_text.strip())] = math.radians(float(offset_text.strip()))
    return offsets


def read_report(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    history = list(data.get("history") or data.get("telemetry") or [])
    if not history:
        raise SystemExit(f"Report senza history/telemetry: {path}")
    return data


def read_report_registration(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    registration = data.get("report_registration", data)
    matrix = registration.get("matrix")
    if registration.get("mode") != "none":
        if not isinstance(matrix, list) or len(matrix) != 2:
            raise SystemExit(f"Registrazione senza matrice 2x3 valida: {path}")
        if any(not isinstance(row, list) or len(row) != 3 for row in matrix):
            raise SystemExit(f"Registrazione senza matrice 2x3 valida: {path}")
    registration = dict(registration)
    registration.setdefault("source", f"loaded_from_{path}")
    return registration


def report_world_bounds(report: dict[str, Any]) -> tuple[float, float, float, float]:
    world = report.get("scene", {}).get("world", {})
    bounds = world.get("bounds", {})
    return (
        float(bounds.get("min_x", 0.0)),
        float(bounds.get("max_x", 1.0)),
        float(bounds.get("min_y", 0.0)),
        float(bounds.get("max_y", 1.0)),
    )


def reference_box_size(config: dict[str, Any]) -> tuple[float, float]:
    box = config.get("reference_box", {})
    width = float(box.get("width_m", 0.0))
    height = float(box.get("height_m", 0.0))
    if width <= 0.0 or height <= 0.0:
        refs = config.get("reference_markers", {})
        points = [item.get("center") for item in refs.values() if isinstance(item, dict)]
        xs = [float(point[0]) for point in points if isinstance(point, list) and len(point) == 2]
        ys = [float(point[1]) for point in points if isinstance(point, list) and len(point) == 2]
        if xs and ys:
            width = max(xs) - min(xs)
            height = max(ys) - min(ys)
    if width <= 0.0 or height <= 0.0:
        raise SystemExit("Il config deve contenere reference_box.width_m/height_m positivi")
    return width, height


def real_to_report_xy(
    x_m: float,
    y_m: float,
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
) -> tuple[float, float]:
    width_m, height_m = config_size
    min_x, max_x, min_y, max_y = world_bounds
    return (
        min_x + (x_m / width_m) * (max_x - min_x),
        min_y + (y_m / height_m) * (max_y - min_y),
    )


def report_to_real_xy(
    x_m: float,
    y_m: float,
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
) -> tuple[float, float]:
    width_m, height_m = config_size
    min_x, max_x, min_y, max_y = world_bounds
    return (
        ((x_m - min_x) / max(max_x - min_x, 1e-9)) * width_m,
        ((y_m - min_y) / max(max_y - min_y, 1e-9)) * height_m,
    )


def transform_report_xy(
    x_m: float,
    y_m: float,
    registration: dict[str, Any] | None,
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
) -> tuple[float, float]:
    if not registration or registration.get("mode") == "none":
        return report_to_real_xy(x_m, y_m, config_size, world_bounds)
    matrix = registration.get("matrix")
    if not isinstance(matrix, list) or len(matrix) != 2:
        return report_to_real_xy(x_m, y_m, config_size, world_bounds)
    return (
        float(matrix[0][0]) * x_m + float(matrix[0][1]) * y_m + float(matrix[0][2]),
        float(matrix[1][0]) * x_m + float(matrix[1][1]) * y_m + float(matrix[1][2]),
    )


def transform_report_yaw(yaw_rad: float, registration: dict[str, Any] | None) -> float:
    if not registration or registration.get("mode") == "none":
        return wrap_angle(yaw_rad)
    matrix = registration.get("matrix")
    if not isinstance(matrix, list) or len(matrix) != 2:
        return wrap_angle(yaw_rad)
    vx = math.cos(yaw_rad)
    vy = math.sin(yaw_rad)
    out_x = float(matrix[0][0]) * vx + float(matrix[0][1]) * vy
    out_y = float(matrix[1][0]) * vx + float(matrix[1][1]) * vy
    if abs(out_x) < 1e-12 and abs(out_y) < 1e-12:
        return wrap_angle(yaw_rad)
    return wrap_angle(math.atan2(out_y, out_x))


def marker_to_robot_center_real_xy(
    marker_x_m: float,
    marker_y_m: float,
    robot_yaw_rad: float,
    robot_to_marker_x_m: float,
    robot_to_marker_y_m: float,
) -> tuple[float, float]:
    cos_yaw = math.cos(robot_yaw_rad)
    sin_yaw = math.sin(robot_yaw_rad)
    offset_world_x = cos_yaw * robot_to_marker_x_m - sin_yaw * robot_to_marker_y_m
    offset_world_y = sin_yaw * robot_to_marker_x_m + cos_yaw * robot_to_marker_y_m
    return marker_x_m - offset_world_x, marker_y_m - offset_world_y


def load_report_series(report: dict[str, Any]) -> dict[str, list[float]]:
    history = list(report.get("history") or report.get("telemetry") or [])
    rows = [
        sample
        for sample in history
        if isinstance(sample.get("time"), (int, float))
        and isinstance(sample.get("position_x"), (int, float))
        and isinstance(sample.get("position_y"), (int, float))
        and isinstance(sample.get("yaw"), (int, float))
    ]
    if len(rows) < 2:
        raise SystemExit("Il report non contiene abbastanza pose interpolabili")

    times = [float(sample["time"]) for sample in rows]
    xs = [float(sample["position_x"]) for sample in rows]
    ys = [float(sample["position_y"]) for sample in rows]
    yaws = unwrap_angles([float(sample["yaw"]) for sample in rows])
    return {"time": times, "x": xs, "y": ys, "yaw": yaws}


def unwrap_angles(values: list[float]) -> list[float]:
    if not values:
        return []
    unwrapped = [values[0]]
    for value in values[1:]:
        previous = unwrapped[-1]
        unwrapped.append(previous + wrap_angle(value - previous))
    return unwrapped


def interp(series: dict[str, list[float]], time_s: float) -> tuple[float, float, float] | None:
    times = series["time"]
    if time_s < times[0] or time_s > times[-1]:
        return None
    lo = 0
    hi = len(times) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if times[mid] < time_s:
            lo = mid + 1
        else:
            hi = mid - 1
    index = max(1, lo)
    t0 = times[index - 1]
    t1 = times[index]
    alpha = 0.0 if t1 <= t0 else (time_s - t0) / (t1 - t0)
    x = series["x"][index - 1] * (1.0 - alpha) + series["x"][index] * alpha
    y = series["y"][index - 1] * (1.0 - alpha) + series["y"][index] * alpha
    yaw = series["yaw"][index - 1] * (1.0 - alpha) + series["yaw"][index] * alpha
    return x, y, yaw


def process_video(
    video_path: Path,
    config: dict[str, Any],
    dictionaries: list[str],
    robot_id: int,
    frame_step: int,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    aruco.require_opencv()
    cv2 = aruco.cv2
    np = aruco.np
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"Impossibile aprire video: {video_path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

    reference_cfg = config.get("reference_markers", {})
    reference_ids = {int(marker_id) for marker_id in reference_cfg.keys()}
    reference_observations: dict[int, list[list[float]]] = {marker_id: [] for marker_id in reference_ids}
    rows: list[dict[str, Any]] = []
    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx % max(frame_step, 1) != 0:
            frame_idx += 1
            continue

        detections = aruco.detect_markers(frame, dictionaries)
        detected_by_id = {det.marker_id: det for det in detections}
        for marker_id in reference_ids:
            det = detected_by_id.get(marker_id)
            if det is not None:
                reference_observations[marker_id].append(det.center_px.tolist())
        homography, used_reference_ids, reprojection_rmse = aruco.estimate_homography(detections, config)
        poses = (
            [aruco.marker_pose_from_homography(det, homography) for det in detections]
            if homography is not None
            else [aruco.marker_pose_pixels(det) for det in detections]
        )
        robot_pose = next((pose for pose in poses if int(pose["id"]) == int(robot_id)), None)
        robot_detection = detected_by_id.get(robot_id)
        row: dict[str, Any] = {
            "frame_idx": frame_idx,
            "video_time_s": frame_idx / fps if fps > 0.0 else None,
            "detected_markers": len(detections),
            "reference_ids": " ".join(str(item) for item in sorted(used_reference_ids)),
            "reference_count": len(used_reference_ids),
            "homography_available": homography is not None,
            "homography_source": "per_frame" if homography is not None else "",
            "reprojection_rmse_m": reprojection_rmse,
            "robot_seen": robot_detection is not None,
            "robot_x_m": None,
            "robot_y_m": None,
            "robot_yaw_rad": None,
            "robot_yaw_deg": None,
            "_robot_corners_px": robot_detection.corners_px.tolist() if robot_detection is not None else None,
        }
        if robot_pose is not None and "center_m" in robot_pose:
            row.update(
                {
                    "robot_x_m": float(robot_pose["center_m"][0]),
                    "robot_y_m": float(robot_pose["center_m"][1]),
                    "robot_yaw_rad": float(robot_pose["yaw_m_rad"]),
                    "robot_yaw_deg": float(robot_pose["yaw_m_deg"]),
                }
            )
        rows.append(row)
        frame_idx += 1
    cap.release()

    global_homography, global_reference_ids, global_rmse = estimate_global_homography(
        reference_observations,
        config,
    )
    if global_homography is not None:
        for row in rows:
            corners_px = row.get("_robot_corners_px")
            if corners_px is None:
                continue
            corners_m = aruco.apply_homography(global_homography, np.asarray(corners_px, dtype=np.float64))
            center_m = corners_m.mean(axis=0)
            axis = corners_m[1] - corners_m[0]
            yaw_m = math.atan2(float(axis[1]), float(axis[0]))
            row.update(
                {
                    "homography_available": True,
                    "homography_source": "global_video",
                    "reprojection_rmse_m": global_rmse,
                    "robot_x_m": float(center_m[0]),
                    "robot_y_m": float(center_m[1]),
                    "robot_yaw_rad": yaw_m,
                    "robot_yaw_deg": math.degrees(wrap_angle(yaw_m)),
                }
            )

    meta = {
        "fps": fps,
        "frame_count": frame_count,
        "duration_s": frame_count / fps if fps > 0.0 else None,
        "width": width,
        "height": height,
        "processed_frames": len(rows),
        "global_homography_available": global_homography is not None,
        "global_reference_ids": global_reference_ids,
        "global_reprojection_rmse_m": global_rmse,
        "global_image_to_map": global_homography.tolist() if global_homography is not None else None,
        "reference_detection_counts": {
            str(marker_id): len(items) for marker_id, items in sorted(reference_observations.items())
        },
    }
    return rows, meta


def estimate_global_homography(
    reference_observations: dict[int, list[list[float]]],
    config: dict[str, Any],
) -> tuple[Any | None, list[int], float | None]:
    aruco.require_opencv()
    cv2 = aruco.cv2
    np = aruco.np
    refs = config.get("reference_markers", {})
    image_points: list[list[float]] = []
    world_points: list[list[float]] = []
    used_ids: list[int] = []
    for marker_id, observations in sorted(reference_observations.items()):
        marker_cfg = refs.get(str(marker_id))
        if not observations or not isinstance(marker_cfg, dict) or "center" not in marker_cfg:
            continue
        obs = np.asarray(observations, dtype=np.float64)
        median_center = np.median(obs, axis=0)
        image_points.append([float(median_center[0]), float(median_center[1])])
        world_points.append([float(marker_cfg["center"][0]), float(marker_cfg["center"][1])])
        used_ids.append(marker_id)
    if len(image_points) < 4:
        return None, used_ids, None
    image_array = np.asarray(image_points, dtype=np.float64).reshape(-1, 1, 2)
    world_array = np.asarray(world_points, dtype=np.float64).reshape(-1, 1, 2)
    homography, _ = cv2.findHomography(image_array, world_array, method=0)
    if homography is None:
        return None, used_ids, None
    projected = aruco.apply_homography(homography, np.asarray(image_points, dtype=np.float64))
    world_flat = np.asarray(world_points, dtype=np.float64)
    rmse = float(np.sqrt(np.mean(np.sum((projected - world_flat) ** 2, axis=1))))
    return homography, used_ids, rmse


def write_pose_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "frame_idx",
        "video_time_s",
        "detected_markers",
        "reference_ids",
        "reference_count",
        "homography_available",
        "homography_source",
        "reprojection_rmse_m",
        "robot_seen",
        "robot_x_m",
        "robot_y_m",
        "robot_yaw_rad",
        "robot_yaw_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def valid_pose_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        row
        for row in rows
        if row.get("homography_available")
        and row.get("robot_seen")
        and isinstance(row.get("robot_x_m"), float)
        and isinstance(row.get("robot_y_m"), float)
        and isinstance(row.get("robot_yaw_rad"), float)
        and isinstance(row.get("video_time_s"), float)
    ]


def split_segments(
    rows: list[dict[str, Any]],
    min_samples: int,
    max_gap_s: float,
    max_jump_m: float,
    max_yaw_jump_rad: float,
) -> list[list[dict[str, Any]]]:
    segments: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    previous: dict[str, Any] | None = None
    for row in rows:
        if previous is not None:
            dt = float(row["video_time_s"]) - float(previous["video_time_s"])
            dx = float(row["robot_x_m"]) - float(previous["robot_x_m"])
            dy = float(row["robot_y_m"]) - float(previous["robot_y_m"])
            dyaw = abs(angle_delta(float(row["robot_yaw_rad"]), float(previous["robot_yaw_rad"])))
            jump = math.hypot(dx, dy)
            if dt > max_gap_s or jump > max_jump_m or dyaw > max_yaw_jump_rad:
                if len(current) >= min_samples:
                    segments.append(current)
                current = []
        current.append(row)
        previous = row
    if len(current) >= min_samples:
        segments.append(current)
    return segments


def segment_points_in_report_frame(
    segment: list[dict[str, Any]],
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
) -> tuple[list[float], list[tuple[float, float]], list[float]]:
    t0 = float(segment[0]["video_time_s"])
    rel_times: list[float] = []
    points: list[tuple[float, float]] = []
    yaws: list[float] = []
    for row in segment:
        rel_times.append(float(row["video_time_s"]) - t0)
        points.append(
            real_to_report_xy(
                float(row["robot_x_m"]),
                float(row["robot_y_m"]),
                config_size,
                world_bounds,
            )
        )
        yaws.append(float(row["robot_yaw_rad"]))
    return rel_times, points, unwrap_angles(yaws)


def segment_points_for_alignment(
    segment: list[dict[str, Any]],
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
    report_registration: dict[str, Any] | None = None,
) -> tuple[list[float], list[tuple[float, float]], list[float]]:
    if report_registration and report_registration.get("mode") != "none":
        t0 = float(segment[0]["video_time_s"])
        rel_times: list[float] = []
        points: list[tuple[float, float]] = []
        yaws: list[float] = []
        for row in segment:
            rel_times.append(float(row["video_time_s"]) - t0)
            points.append((float(row["robot_x_m"]), float(row["robot_y_m"])))
            yaws.append(float(row["robot_yaw_rad"]))
        return rel_times, points, unwrap_angles(yaws)
    return segment_points_in_report_frame(segment, config_size, world_bounds)


def alignment_cost(
    rel_times: list[float],
    points: list[tuple[float, float]],
    yaws: list[float],
    report_series: dict[str, list[float]],
    log_start_s: float,
    report_registration: dict[str, Any] | None = None,
    config_size: tuple[float, float] | None = None,
    world_bounds: tuple[float, float, float, float] | None = None,
) -> float | None:
    report_samples: list[tuple[float, float, float]] = []
    for rel_time in rel_times:
        sample = interp(report_series, log_start_s + rel_time)
        if sample is None:
            return None
        if report_registration and report_registration.get("mode") != "none":
            if config_size is None or world_bounds is None:
                return None
            x_m, y_m = transform_report_xy(
                sample[0],
                sample[1],
                report_registration,
                config_size,
                world_bounds,
            )
            yaw_rad = transform_report_yaw(sample[2], report_registration)
            sample = (x_m, y_m, yaw_rad)
        report_samples.append(sample)

    gt0 = points[0]
    report0 = report_samples[0]
    pos_abs_sq = 0.0
    pos_rel_sq = 0.0
    yaw_rel_sq = 0.0
    for point, yaw, report_sample in zip(points, yaws, report_samples):
        abs_dx = point[0] - report_sample[0]
        abs_dy = point[1] - report_sample[1]
        pos_abs_sq += abs_dx * abs_dx + abs_dy * abs_dy
        rel_dx = (point[0] - gt0[0]) - (report_sample[0] - report0[0])
        rel_dy = (point[1] - gt0[1]) - (report_sample[1] - report0[1])
        pos_rel_sq += rel_dx * rel_dx + rel_dy * rel_dy
        yaw_rel_sq += angle_delta(yaw - yaws[0], report_sample[2] - report0[2]) ** 2

    n = max(len(points), 1)
    pos_abs_rms = math.sqrt(pos_abs_sq / n)
    pos_rel_rms = math.sqrt(pos_rel_sq / n)
    yaw_rel_rms = math.sqrt(yaw_rel_sq / n)
    return 0.65 * pos_abs_rms + pos_rel_rms + 0.035 * yaw_rel_rms


def choose_subsample(length: int, max_items: int = 100) -> list[int]:
    if length <= max_items:
        return list(range(length))
    step = (length - 1) / float(max_items - 1)
    return sorted({int(round(i * step)) for i in range(max_items)})


def align_segment(
    segment: list[dict[str, Any]],
    report_series: dict[str, list[float]],
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
    search_step_s: float,
    report_registration: dict[str, Any] | None = None,
) -> dict[str, Any]:
    candidates = segment_alignment_candidates(
        segment,
        report_series,
        config_size,
        world_bounds,
        search_step_s,
        report_registration,
    )
    if not candidates:
        return {
            "aligned": False,
            "video_start_s": float(segment[0]["video_time_s"]),
            "video_end_s": float(segment[-1]["video_time_s"]),
            "samples": len(segment),
        }
    return candidates[0]


def manual_align_segments(
    segments: list[list[dict[str, Any]]],
    report_series: dict[str, list[float]],
    video_log_start_s: float,
    report_log_start_s: float,
) -> list[dict[str, Any]]:
    report_min = report_series["time"][0]
    report_max = report_series["time"][-1]
    alignments: list[dict[str, Any]] = []
    for segment in segments:
        video_start = float(segment[0]["video_time_s"])
        video_end = float(segment[-1]["video_time_s"])
        duration = video_end - video_start
        log_start = report_log_start_s + (video_start - video_log_start_s)
        log_end = log_start + duration
        aligned = log_end >= report_min and log_start <= report_max
        alignments.append(
            {
                "aligned": aligned,
                "log_start_s": log_start,
                "video_start_s": video_start,
                "video_end_s": video_end,
                "duration_s": duration,
                "samples": len(segment),
                "frame_start": int(segment[0]["frame_idx"]),
                "frame_end": int(segment[-1]["frame_idx"]),
                "source": "manual_video_log_start",
            }
        )
    return alignments


def segment_alignment_candidates(
    segment: list[dict[str, Any]],
    report_series: dict[str, list[float]],
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
    search_step_s: float,
    report_registration: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    rel_times, points, yaws = segment_points_for_alignment(
        segment,
        config_size,
        world_bounds,
        report_registration,
    )
    sample_indices = choose_subsample(len(rel_times))
    rel_sub = [rel_times[i] for i in sample_indices]
    points_sub = [points[i] for i in sample_indices]
    yaws_sub = [yaws[i] for i in sample_indices]

    duration = rel_times[-1] if rel_times else 0.0
    report_min = report_series["time"][0]
    report_max = report_series["time"][-1]
    candidates: list[dict[str, Any]] = []
    start = report_min
    latest_start = report_max - duration
    while start <= latest_start + 1e-9:
        cost = alignment_cost(
            rel_sub,
            points_sub,
            yaws_sub,
            report_series,
            start,
            report_registration,
            config_size,
            world_bounds,
        )
        if cost is not None:
            candidates.append(
                {
                    "aligned": True,
                    "log_start_s": start,
                    "cost": cost,
                    "video_start_s": float(segment[0]["video_time_s"]),
                    "video_end_s": float(segment[-1]["video_time_s"]),
                    "duration_s": duration,
                    "samples": len(segment),
                    "frame_start": int(segment[0]["frame_idx"]),
                    "frame_end": int(segment[-1]["frame_idx"]),
                }
            )
        start += max(search_step_s, 0.01)
    return sorted(candidates, key=lambda item: float(item["cost"]))


def choose_monotonic_alignments(
    candidates_by_segment: list[list[dict[str, Any]]],
) -> list[dict[str, Any]]:
    if not candidates_by_segment:
        return []
    trimmed = [items for items in candidates_by_segment]
    dp: list[list[tuple[float, int | None]]] = []
    for segment_index, candidates in enumerate(trimmed):
        segment_dp: list[tuple[float, int | None]] = []
        for candidate_index, candidate in enumerate(candidates):
            weight = max(float(candidate.get("samples", 1)), 1.0)
            local_cost = float(candidate["cost"]) * weight
            if segment_index == 0:
                segment_dp.append((local_cost, None))
                continue
            best_previous: tuple[float, int] | None = None
            current_start = float(candidate["log_start_s"])
            for previous_index, previous in enumerate(trimmed[segment_index - 1]):
                previous_start = float(previous["log_start_s"])
                previous_end = previous_start + float(previous.get("duration_s", 0.0))
                if current_start + 1e-9 < previous_end:
                    continue
                previous_score = dp[segment_index - 1][previous_index][0]
                if best_previous is None or previous_score < best_previous[0]:
                    best_previous = (previous_score, previous_index)
            if best_previous is None:
                segment_dp.append((float("inf"), None))
            else:
                segment_dp.append((best_previous[0] + local_cost, best_previous[1]))
        dp.append(segment_dp)

    if not dp[-1]:
        return []
    final_index = min(range(len(dp[-1])), key=lambda index: dp[-1][index][0])
    if not math.isfinite(dp[-1][final_index][0]):
        return [items[0] for items in trimmed if items]

    selected_reversed: list[dict[str, Any]] = []
    current_index: int | None = final_index
    for segment_index in range(len(trimmed) - 1, -1, -1):
        if current_index is None:
            break
        selected_reversed.append(trimmed[segment_index][current_index])
        current_index = dp[segment_index][current_index][1]
    selected = list(reversed(selected_reversed))
    if len(selected) != len(trimmed):
        return [items[0] for items in trimmed if items]
    return selected


def circular_mean(values: list[float]) -> float:
    if not values:
        return 0.0
    s = sum(math.sin(value) for value in values)
    c = sum(math.cos(value) for value in values)
    return math.atan2(s, c)


def registration_residuals(
    pairs: list[tuple[tuple[float, float], tuple[float, float]]],
    registration: dict[str, Any],
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
) -> list[float]:
    return [
        math.hypot(
            transform_report_xy(src[0], src[1], registration, config_size, world_bounds)[0] - dst[0],
            transform_report_xy(src[0], src[1], registration, config_size, world_bounds)[1] - dst[1],
        )
        for src, dst in pairs
    ]


def fit_similarity_registration(
    pairs: list[tuple[tuple[float, float], tuple[float, float]]],
) -> dict[str, Any] | None:
    if len(pairs) < 2:
        return None
    src_x = [src[0] for src, _ in pairs]
    src_y = [src[1] for src, _ in pairs]
    dst_x = [dst[0] for _, dst in pairs]
    dst_y = [dst[1] for _, dst in pairs]
    src_mean_x = statistics.fmean(src_x)
    src_mean_y = statistics.fmean(src_y)
    dst_mean_x = statistics.fmean(dst_x)
    dst_mean_y = statistics.fmean(dst_y)

    numerator_a = 0.0
    numerator_b = 0.0
    denominator = 0.0
    for sx, sy, dx, dy in zip(src_x, src_y, dst_x, dst_y):
        psx = sx - src_mean_x
        psy = sy - src_mean_y
        qx = dx - dst_mean_x
        qy = dy - dst_mean_y
        numerator_a += qx * psx + qy * psy
        numerator_b += qy * psx - qx * psy
        denominator += psx * psx + psy * psy
    if denominator <= 1e-12:
        return None

    a = numerator_a / denominator
    b = numerator_b / denominator
    tx = dst_mean_x - (a * src_mean_x - b * src_mean_y)
    ty = dst_mean_y - (b * src_mean_x + a * src_mean_y)
    return {
        "mode": "similarity",
        "matrix": [[a, -b, tx], [b, a, ty]],
        "scale": math.hypot(a, b),
        "rotation_deg": math.degrees(math.atan2(b, a)),
    }


def fit_affine_registration(
    pairs: list[tuple[tuple[float, float], tuple[float, float]]],
) -> dict[str, Any] | None:
    if len(pairs) < 3:
        return None
    aruco.require_opencv()
    np = aruco.np
    lhs: list[list[float]] = []
    rhs: list[float] = []
    for (sx, sy), (dx, dy) in pairs:
        lhs.append([sx, sy, 1.0, 0.0, 0.0, 0.0])
        lhs.append([0.0, 0.0, 0.0, sx, sy, 1.0])
        rhs.append(dx)
        rhs.append(dy)
    solution, *_ = np.linalg.lstsq(
        np.asarray(lhs, dtype=np.float64),
        np.asarray(rhs, dtype=np.float64),
        rcond=None,
    )
    return {
        "mode": "affine",
        "matrix": [
            [float(solution[0]), float(solution[1]), float(solution[2])],
            [float(solution[3]), float(solution[4]), float(solution[5])],
        ],
    }


def fit_initial_rigid_registration(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not rows:
        return None
    row = min(rows, key=lambda item: float(item["log_time_s"]))
    theta = angle_delta(
        float(row["gt_yaw_with_fitted_offset_rad"]),
        float(row["log_yaw_rad"]),
    )
    c = math.cos(theta)
    s = math.sin(theta)
    log_x = float(row["log_x"])
    log_y = float(row["log_y"])
    gt_x = float(row["gt_robot_center_x_m"])
    gt_y = float(row["gt_robot_center_y_m"])
    tx = gt_x - (c * log_x - s * log_y)
    ty = gt_y - (s * log_x + c * log_y)
    return {
        "mode": "initial_rigid",
        "matrix": [[c, -s, tx], [s, c, ty]],
        "scale": 1.0,
        "rotation_deg": math.degrees(theta),
        "anchor": {
            "segment": int(row["segment"]),
            "frame_idx": int(row["frame_idx"]),
            "video_time_s": float(row["video_time_s"]),
            "log_time_s": float(row["log_time_s"]),
            "log_x": log_x,
            "log_y": log_y,
            "log_yaw_rad": float(row["log_yaw_rad"]),
            "gt_robot_center_x_m": gt_x,
            "gt_robot_center_y_m": gt_y,
            "gt_yaw_rad": float(row["gt_yaw_with_fitted_offset_rad"]),
        },
    }


def fit_report_registration(
    rows: list[dict[str, Any]],
    mode: str,
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
) -> dict[str, Any]:
    if mode == "none":
        return {"mode": "none", "source": "arena_scale_from_report_bounds"}
    if mode == "initial_rigid":
        registration = fit_initial_rigid_registration(rows)
        if registration is None:
            return {
                "mode": "none",
                "source": "fallback_after_failed_initial_rigid",
                "fit_point_count": len(rows),
            }
        pairs = [
            (
                (float(row["log_x"]), float(row["log_y"])),
                (float(row["gt_robot_center_x_m"]), float(row["gt_robot_center_y_m"])),
            )
            for row in rows
        ]
        residuals = registration_residuals(pairs, registration, config_size, world_bounds)
        registration["source"] = "anchored_to_first_aligned_pose"
        registration["fit_point_count"] = 1
        registration["residual_m"] = stats(residuals)
        return registration

    pairs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for row in rows:
        pairs.append(
            (
                (float(row["log_x"]), float(row["log_y"])),
                (float(row["gt_robot_center_x_m"]), float(row["gt_robot_center_y_m"])),
            )
        )
    fitter = fit_similarity_registration if mode == "similarity" else fit_affine_registration
    registration = fitter(pairs)
    if registration is None:
        return {
            "mode": "none",
            "source": f"fallback_after_failed_{mode}_fit",
            "fit_point_count": len(pairs),
        }

    residuals = registration_residuals(pairs, registration, config_size, world_bounds)
    registration["source"] = "fitted_from_aligned_video_log_samples"
    registration["fit_point_count"] = len(pairs)
    registration["residual_m"] = stats(residuals)
    return registration


def apply_report_registration_to_rows(
    rows: list[dict[str, Any]],
    alignments: list[dict[str, Any]],
    registration: dict[str, Any],
    report_yaw_offset_rad: float,
    report_yaw_offset_by_segment_rad: dict[int, float],
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
) -> None:
    by_segment: dict[int, dict[str, list[float]]] = {}
    for row in rows:
        log_real_x, log_real_y = transform_report_xy(
            float(row["log_x"]),
            float(row["log_y"]),
            registration,
            config_size,
            world_bounds,
        )
        segment = int(row["segment"])
        row_report_yaw_offset_rad = report_yaw_offset_by_segment_rad.get(segment, report_yaw_offset_rad)
        log_real_yaw = wrap_angle(
            transform_report_yaw(float(row["log_yaw_rad"]), registration) + row_report_yaw_offset_rad
        )
        pos_error_real = math.hypot(
            float(row["gt_robot_center_x_m"]) - log_real_x,
            float(row["gt_robot_center_y_m"]) - log_real_y,
        )
        yaw_error = abs(angle_delta(float(row["gt_yaw_with_fitted_offset_rad"]), log_real_yaw))
        row["log_real_x_m"] = log_real_x
        row["log_real_y_m"] = log_real_y
        row["log_real_yaw_rad"] = log_real_yaw
        row["report_yaw_offset_rad"] = row_report_yaw_offset_rad
        row["report_registration_mode"] = registration.get("mode", "none")
        row["position_error_robot_center_real_m"] = pos_error_real
        row["yaw_error_with_fitted_offset_rad"] = yaw_error
        row["yaw_error_with_fitted_offset_deg"] = math.degrees(yaw_error)

        bucket = by_segment.setdefault(
            segment,
            {
                "position_error_uncorrected_marker_m": [],
                "position_error_robot_center_with_fitted_yaw_offset_m": [],
                "position_error_robot_center_real_m": [],
                "yaw_error_with_fitted_offset_deg": [],
            },
        )
        bucket["position_error_uncorrected_marker_m"].append(float(row["position_error_uncorrected_marker_m"]))
        bucket["position_error_robot_center_with_fitted_yaw_offset_m"].append(
            float(row["position_error_robot_center_with_fitted_yaw_offset_m"])
        )
        bucket["position_error_robot_center_real_m"].append(pos_error_real)
        bucket["yaw_error_with_fitted_offset_deg"].append(math.degrees(yaw_error))

    for segment, values_by_key in by_segment.items():
        if segment >= len(alignments):
            continue
        for key, values in values_by_key.items():
            alignments[segment][key] = stats(values)


def build_alignment_rows(
    segments: list[list[dict[str, Any]]],
    alignments: list[dict[str, Any]],
    report_series: dict[str, list[float]],
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
    robot_to_marker_offset_m: tuple[float, float],
    fixed_marker_to_robot_yaw_offset_rad: float | None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    rows: list[dict[str, Any]] = []
    enriched_alignments: list[dict[str, Any]] = []
    for segment_index, (segment, alignment) in enumerate(zip(segments, alignments)):
        enriched = dict(alignment)
        if not alignment.get("aligned"):
            enriched_alignments.append(enriched)
            continue

        yaw_offsets: list[float] = []
        provisional: list[dict[str, Any]] = []
        for row in segment:
            video_time = float(row["video_time_s"])
            rel_time = video_time - float(alignment["video_start_s"])
            log_time = float(alignment["log_start_s"]) + rel_time
            report_sample = interp(report_series, log_time)
            if report_sample is None:
                continue
            gt_x, gt_y = real_to_report_xy(
                float(row["robot_x_m"]),
                float(row["robot_y_m"]),
                config_size,
                world_bounds,
            )
            marker_yaw = float(row["robot_yaw_rad"])
            yaw_offsets.append(angle_delta(report_sample[2], marker_yaw))
            provisional.append(
                {
                    "segment": segment_index,
                    "frame_idx": int(row["frame_idx"]),
                    "video_time_s": video_time,
                    "log_time_s": log_time,
                    "gt_marker_x_m": float(row["robot_x_m"]),
                    "gt_marker_y_m": float(row["robot_y_m"]),
                    "gt_marker_yaw_rad": marker_yaw,
                    "gt_marker_plan_x": gt_x,
                    "gt_marker_plan_y": gt_y,
                    "log_x": report_sample[0],
                    "log_y": report_sample[1],
                    "log_yaw_rad": wrap_angle(report_sample[2]),
                }
            )

        yaw_offset = (
            fixed_marker_to_robot_yaw_offset_rad
            if fixed_marker_to_robot_yaw_offset_rad is not None
            else circular_mean(yaw_offsets)
        )
        enriched["yaw_marker_to_report_offset_rad"] = yaw_offset
        enriched["yaw_marker_to_report_offset_deg"] = math.degrees(yaw_offset)
        enriched["yaw_offset_source"] = "fixed_marker_to_robot" if fixed_marker_to_robot_yaw_offset_rad is not None else "fitted_to_report"
        pos_errors: list[float] = []
        corrected_pos_errors: list[float] = []
        corrected_pos_errors_real: list[float] = []
        yaw_errors: list[float] = []
        for item in provisional:
            dx = item["gt_marker_plan_x"] - item["log_x"]
            dy = item["gt_marker_plan_y"] - item["log_y"]
            pos_error = math.hypot(dx, dy)
            corrected_yaw = wrap_angle(item["gt_marker_yaw_rad"] + yaw_offset)
            robot_real_x, robot_real_y = marker_to_robot_center_real_xy(
                item["gt_marker_x_m"],
                item["gt_marker_y_m"],
                corrected_yaw,
                robot_to_marker_offset_m[0],
                robot_to_marker_offset_m[1],
            )
            robot_plan_x, robot_plan_y = real_to_report_xy(
                robot_real_x,
                robot_real_y,
                config_size,
                world_bounds,
            )
            log_real_x, log_real_y = report_to_real_xy(
                item["log_x"],
                item["log_y"],
                config_size,
                world_bounds,
            )
            corrected_pos_error = math.hypot(robot_plan_x - item["log_x"], robot_plan_y - item["log_y"])
            corrected_pos_error_real = math.hypot(robot_real_x - log_real_x, robot_real_y - log_real_y)
            yaw_error = abs(angle_delta(corrected_yaw, item["log_yaw_rad"]))
            item["gt_robot_center_x_m"] = robot_real_x
            item["gt_robot_center_y_m"] = robot_real_y
            item["gt_robot_center_plan_x"] = robot_plan_x
            item["gt_robot_center_plan_y"] = robot_plan_y
            item["log_real_x_m"] = log_real_x
            item["log_real_y_m"] = log_real_y
            item["gt_yaw_with_fitted_offset_rad"] = corrected_yaw
            item["position_error_uncorrected_marker_m"] = pos_error
            item["position_error_robot_center_with_fitted_yaw_offset_m"] = corrected_pos_error
            item["position_error_robot_center_real_m"] = corrected_pos_error_real
            item["yaw_error_with_fitted_offset_rad"] = yaw_error
            item["yaw_error_with_fitted_offset_deg"] = math.degrees(yaw_error)
            pos_errors.append(pos_error)
            corrected_pos_errors.append(corrected_pos_error)
            corrected_pos_errors_real.append(corrected_pos_error_real)
            yaw_errors.append(math.degrees(yaw_error))
            rows.append(item)
        enriched["position_error_uncorrected_marker_m"] = stats(pos_errors)
        enriched["position_error_robot_center_with_fitted_yaw_offset_m"] = stats(corrected_pos_errors)
        enriched["position_error_robot_center_real_m"] = stats(corrected_pos_errors_real)
        enriched["yaw_error_with_fitted_offset_deg"] = stats(yaw_errors)
        enriched_alignments.append(enriched)
    return rows, enriched_alignments


def write_alignment_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "segment",
        "frame_idx",
        "video_time_s",
        "log_time_s",
        "gt_marker_x_m",
        "gt_marker_y_m",
        "gt_marker_yaw_rad",
        "gt_marker_plan_x",
        "gt_marker_plan_y",
        "gt_robot_center_x_m",
        "gt_robot_center_y_m",
        "gt_robot_center_plan_x",
        "gt_robot_center_plan_y",
        "log_real_x_m",
        "log_real_y_m",
        "log_real_yaw_rad",
        "report_yaw_offset_rad",
        "report_registration_mode",
        "log_x",
        "log_y",
        "log_yaw_rad",
        "gt_yaw_with_fitted_offset_rad",
        "position_error_uncorrected_marker_m",
        "position_error_robot_center_with_fitted_yaw_offset_m",
        "position_error_robot_center_real_m",
        "yaw_error_with_fitted_offset_rad",
        "yaw_error_with_fitted_offset_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_svg_trajectory(
    path: Path,
    report_series: dict[str, list[float]],
    alignment_rows: list[dict[str, Any]],
    world_bounds: tuple[float, float, float, float],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    min_x, max_x, min_y, max_y = world_bounds
    width = 900
    height = 900
    pad = 70

    def px(point: tuple[float, float]) -> tuple[float, float]:
        x, y = point
        sx = pad + (x - min_x) / max(max_x - min_x, 1e-9) * (width - 2 * pad)
        sy = pad + (y - min_y) / max(max_y - min_y, 1e-9) * (height - 2 * pad)
        return sx, sy

    report_points = [px((x, y)) for x, y in zip(report_series["x"], report_series["y"])]
    gt_points = [
        px(
            (
                float(row.get("gt_robot_center_plan_x", row["gt_marker_plan_x"])),
                float(row.get("gt_robot_center_plan_y", row["gt_marker_plan_y"])),
            )
        )
        for row in alignment_rows
    ]

    def polyline(points: list[tuple[float, float]]) -> str:
        return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<rect x="{pad}" y="{pad}" width="{width - 2 * pad}" height="{height - 2 * pad}" fill="#f8f8f8" stroke="#333" stroke-width="2"/>',
        '<text x="24" y="34" font-family="sans-serif" font-size="18" fill="#222">ArUco marker trajectory aligned to hardware report</text>',
        '<text x="24" y="58" font-family="sans-serif" font-size="13" fill="#555">gray: report estimate, orange: ArUco-derived robot center scaled to planner map</text>',
    ]
    if report_points:
        lines.append(f'<polyline points="{polyline(report_points)}" fill="none" stroke="#777" stroke-width="3" opacity="0.75"/>')
    if gt_points:
        lines.append(f'<polyline points="{polyline(gt_points)}" fill="none" stroke="#f28c28" stroke-width="3" opacity="0.90"/>')
        for x, y in gt_points[:: max(1, len(gt_points) // 45)]:
            lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3" fill="#f28c28"/>')
    lines.append("</svg>")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_overlay_video(
    video_path: Path,
    output_path: Path,
    alignment_rows: list[dict[str, Any]],
    config_size: tuple[float, float],
    world_bounds: tuple[float, float, float, float],
    image_to_map: list[list[float]] | None,
) -> None:
    aruco.require_opencv()
    cv2 = aruco.cv2
    np = aruco.np
    if not alignment_rows:
        return
    row_by_frame = {int(row["frame_idx"]): row for row in alignment_rows}
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"Impossibile aprire video per overlay: {video_path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 25.0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
    if not writer.isOpened():
        cap.release()
        raise SystemExit(f"Impossibile creare video overlay: {output_path}")

    map_to_image = None
    if image_to_map is not None:
        map_to_image = np.linalg.inv(np.asarray(image_to_map, dtype=np.float64))

    arena_w, arena_h = config_size

    def project(point_m: tuple[float, float]) -> tuple[int, int]:
        if map_to_image is not None:
            point = np.asarray([point_m[0], point_m[1], 1.0], dtype=np.float64)
            mapped = map_to_image @ point
            mapped /= mapped[2]
            return int(round(float(mapped[0]))), int(round(float(mapped[1])))
        return int(round(point_m[0] / arena_w * width)), int(round(point_m[1] / arena_h * height))

    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        row = row_by_frame.get(frame_idx)
        if row is not None:
            marker_x = float(row["gt_marker_x_m"])
            marker_y = float(row["gt_marker_y_m"])
            gt_x = float(row.get("gt_robot_center_x_m", marker_x))
            gt_y = float(row.get("gt_robot_center_y_m", marker_y))
            log_real_x = float(row["log_real_x_m"])
            log_real_y = float(row["log_real_y_m"])
            yaw = float(row["gt_yaw_with_fitted_offset_rad"])
            log_yaw = float(row.get("log_real_yaw_rad", row["log_yaw_rad"]))

            gt_px = project((gt_x, gt_y))
            marker_px = project((marker_x, marker_y))
            log_px = project((log_real_x, log_real_y))
            arrow_m = 0.14
            gt_tip = project((gt_x + math.cos(yaw) * arrow_m, gt_y + math.sin(yaw) * arrow_m))
            log_tip = project((log_real_x + math.cos(log_yaw) * arrow_m, log_real_y + math.sin(log_yaw) * arrow_m))
            cv2.circle(frame, marker_px, 5, (0, 210, 255), 1, cv2.LINE_AA)
            cv2.circle(frame, gt_px, 8, (0, 165, 255), -1, cv2.LINE_AA)
            cv2.arrowedLine(frame, gt_px, gt_tip, (0, 165, 255), 3, cv2.LINE_AA, tipLength=0.25)
            cv2.circle(frame, log_px, 7, (255, 90, 90), -1, cv2.LINE_AA)
            cv2.arrowedLine(frame, log_px, log_tip, (255, 90, 90), 2, cv2.LINE_AA, tipLength=0.25)
            text = (
                f"seg {row['segment']} log {float(row['log_time_s']):.2f}s "
                f"err {float(row['position_error_robot_center_real_m']):.2f}m "
                f"yaw {float(row['yaw_error_with_fitted_offset_deg']):.1f}deg"
            )
            cv2.putText(
                frame,
                text,
                (24, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.82,
                (20, 20, 20),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                text,
                (24, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.82,
                (245, 245, 245),
                2,
                cv2.LINE_AA,
            )
        writer.write(frame)
        frame_idx += 1
    cap.release()
    writer.release()


def write_map_video(
    video_path: Path,
    output_path: Path,
    alignment_rows: list[dict[str, Any]],
    config_size: tuple[float, float],
    image_to_map: list[list[float]] | None,
    pixels_per_meter: float,
    padding_px: int,
) -> None:
    aruco.require_opencv()
    cv2 = aruco.cv2
    np = aruco.np
    if not alignment_rows or image_to_map is None:
        return

    arena_w, arena_h = config_size
    min_x, max_x, min_y, max_y = 0.0, arena_w, 0.0, arena_h
    width_px = int(math.ceil(arena_w * pixels_per_meter)) + 2 * padding_px
    height_px = int(math.ceil(arena_h * pixels_per_meter)) + 2 * padding_px
    width_px = max(width_px, 320)
    height_px = max(height_px, 320)
    world_to_canvas = aruco.world_to_canvas_matrix(min_x, min_y, pixels_per_meter, padding_px)
    image_to_canvas = world_to_canvas @ np.asarray(image_to_map, dtype=np.float64)

    row_by_frame = {int(row["frame_idx"]): row for row in alignment_rows}
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"Impossibile aprire video per mappa: {video_path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 25.0)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width_px, height_px))
    if not writer.isOpened():
        cap.release()
        raise SystemExit(f"Impossibile creare video mappa: {output_path}")

    def canvas_point(point_m: tuple[float, float]) -> tuple[int, int]:
        return aruco.world_point_to_canvas(point_m, world_to_canvas)

    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        warped = cv2.warpPerspective(
            frame,
            image_to_canvas,
            (width_px, height_px),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(248, 248, 248),
        )
        grid = np.full_like(warped, 248)
        aruco.draw_map_grid(grid, min_x, max_x, min_y, max_y, world_to_canvas)
        map_view = cv2.addWeighted(grid, 0.35, warped, 0.65, 0.0)
        aruco.draw_map_grid(map_view, min_x, max_x, min_y, max_y, world_to_canvas)

        row = row_by_frame.get(frame_idx)
        if row is not None:
            marker = (float(row["gt_marker_x_m"]), float(row["gt_marker_y_m"]))
            gt = (
                float(row.get("gt_robot_center_x_m", marker[0])),
                float(row.get("gt_robot_center_y_m", marker[1])),
            )
            log = (float(row["log_real_x_m"]), float(row["log_real_y_m"]))
            yaw = float(row["gt_yaw_with_fitted_offset_rad"])
            log_yaw = float(row.get("log_real_yaw_rad", row["log_yaw_rad"]))
            gt_px = canvas_point(gt)
            marker_px = canvas_point(marker)
            log_px = canvas_point(log)
            arrow_m = 0.13
            gt_tip = canvas_point((gt[0] + math.cos(yaw) * arrow_m, gt[1] + math.sin(yaw) * arrow_m))
            log_tip = canvas_point((log[0] + math.cos(log_yaw) * arrow_m, log[1] + math.sin(log_yaw) * arrow_m))
            cv2.circle(map_view, marker_px, 5, (0, 210, 255), 1, cv2.LINE_AA)
            cv2.circle(map_view, gt_px, 8, (0, 165, 255), -1, cv2.LINE_AA)
            cv2.arrowedLine(map_view, gt_px, gt_tip, (0, 165, 255), 3, cv2.LINE_AA, tipLength=0.25)
            cv2.circle(map_view, log_px, 7, (255, 90, 90), -1, cv2.LINE_AA)
            cv2.arrowedLine(map_view, log_px, log_tip, (255, 90, 90), 2, cv2.LINE_AA, tipLength=0.25)
            label = (
                f"seg {row['segment']} log {float(row['log_time_s']):.2f}s | "
                f"pos err {float(row['position_error_robot_center_real_m']):.2f}m | "
                f"yaw err {float(row['yaw_error_with_fitted_offset_deg']):.1f}deg"
            )
            offset = float(row["gt_yaw_with_fitted_offset_rad"]) - float(row["gt_marker_yaw_rad"])
            report_yaw_offset = math.degrees(float(row.get("report_yaw_offset_rad", 0.0)))
            registration_mode = row.get("report_registration_mode", "none")
            label2 = (
                f"orange: ArUco robot center | blue: registered report ({registration_mode}) | "
                f"marker yaw offset {math.degrees(wrap_angle(offset)):.1f}deg | "
                f"report yaw offset {report_yaw_offset:.1f}deg"
            )
            for text, y in [(label, 30), (label2, 56)]:
                cv2.putText(map_view, text, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (20, 20, 20), 3, cv2.LINE_AA)
                cv2.putText(map_view, text, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (245, 245, 245), 1, cv2.LINE_AA)
        else:
            text = "No ArUco robot pose for this frame"
            cv2.putText(map_view, text, (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (20, 20, 20), 3, cv2.LINE_AA)
            cv2.putText(map_view, text, (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (245, 245, 245), 1, cv2.LINE_AA)

        writer.write(map_view)
        frame_idx += 1
    cap.release()
    writer.release()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    config = aruco.load_config(args.config)
    if not config:
        raise SystemExit(f"Config vuoto o mancante: {args.config}")
    report_yaw_offset_by_segment_rad = parse_segment_yaw_offsets(args.report_yaw_offset_by_segment)
    robot_id = args.robot_id if args.robot_id is not None else int(config.get("robot_id", 0))
    dictionaries = args.dictionary or aruco.dictionary_names_from_config(config)
    report = read_report(args.report)
    report_series = load_report_series(report)
    world_bounds = report_world_bounds(report)
    config_size = reference_box_size(config)
    provided_report_registration = (
        read_report_registration(args.report_registration_file)
        if args.report_registration_file is not None
        else None
    )

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.video.stem

    pose_rows, video_meta = process_video(
        args.video,
        config,
        dictionaries,
        robot_id,
        max(args.frame_step, 1),
    )
    pose_csv = output_dir / f"{stem}_aruco_video_poses.csv"
    write_pose_csv(pose_csv, pose_rows)

    valid_rows = valid_pose_rows(pose_rows)
    if args.video_filter_start_s is not None:
        valid_rows = [
            row
            for row in valid_rows
            if row.get("video_time_s") is not None
            and float(row["video_time_s"]) >= float(args.video_filter_start_s)
        ]
    if args.video_filter_end_s is not None:
        valid_rows = [
            row
            for row in valid_rows
            if row.get("video_time_s") is not None
            and float(row["video_time_s"]) <= float(args.video_filter_end_s)
        ]
    if args.video_log_start_s is not None:
        video_log_start_s = float(args.video_log_start_s)
        report_log_start_s = float(args.report_log_start_s)
        report_end_s = float(report_series["time"][-1])
        video_log_end_s = video_log_start_s + max(report_end_s - report_log_start_s, 0.0) + 0.50
        valid_rows = [
            row
            for row in valid_rows
            if row.get("video_time_s") is not None
            and video_log_start_s <= float(row["video_time_s"]) <= video_log_end_s
        ]
    segments = split_segments(
        valid_rows,
        max(args.min_segment_samples, 1),
        max(args.max_gap_s, 0.0),
        max(args.max_jump_m, 0.0),
        max(args.max_yaw_jump_rad, 0.0),
    )
    if args.video_log_start_s is not None:
        alignments = manual_align_segments(
            segments,
            report_series,
            float(args.video_log_start_s),
            float(args.report_log_start_s),
        )
    elif args.alignment_mode == "independent":
        alignments = [
            align_segment(
                segment,
                report_series,
                config_size,
                world_bounds,
                max(args.search_step_s, 0.01),
                provided_report_registration,
            )
            for segment in segments
        ]
    else:
        candidates_by_segment = [
            segment_alignment_candidates(
                segment,
                report_series,
                config_size,
                world_bounds,
                max(args.search_step_s, 0.01),
                provided_report_registration,
            )
            for segment in segments
        ]
        alignments = choose_monotonic_alignments(candidates_by_segment)
    alignment_rows, alignments = build_alignment_rows(
        segments,
        alignments,
        report_series,
        config_size,
        world_bounds,
        (float(args.robot_to_marker_x_m), float(args.robot_to_marker_y_m)),
        math.radians(float(args.marker_to_robot_yaw_offset_deg))
        if args.marker_to_robot_yaw_offset_deg is not None
        else None,
    )
    report_registration = (
        provided_report_registration
        if provided_report_registration is not None
        else fit_report_registration(
            alignment_rows,
            args.report_registration,
            config_size,
            world_bounds,
        )
    )
    apply_report_registration_to_rows(
        alignment_rows,
        alignments,
        report_registration,
        math.radians(float(args.report_yaw_offset_deg)),
        report_yaw_offset_by_segment_rad,
        config_size,
        world_bounds,
    )

    alignment_csv = output_dir / f"{stem}_alignment.csv"
    write_alignment_csv(alignment_csv, alignment_rows)
    trajectory_svg = output_dir / f"{stem}_trajectory_alignment.svg"
    write_svg_trajectory(trajectory_svg, report_series, alignment_rows, world_bounds)
    overlay_video = output_dir / f"{stem}_overlay_alignment.mp4"
    if args.write_overlay_video:
        write_overlay_video(
            args.video,
            overlay_video,
            alignment_rows,
            config_size,
            world_bounds,
            video_meta.get("global_image_to_map"),
        )
    map_video = output_dir / f"{stem}_map_alignment.mp4"
    if args.write_map_video:
        write_map_video(
            args.video,
            map_video,
            alignment_rows,
            config_size,
            video_meta.get("global_image_to_map"),
            max(float(args.map_pixels_per_meter), 1.0),
            max(int(args.map_padding_px), 0),
        )

    all_pos_errors = [float(row["position_error_uncorrected_marker_m"]) for row in alignment_rows]
    all_robot_center_pos_errors = [
        float(row["position_error_robot_center_with_fitted_yaw_offset_m"]) for row in alignment_rows
    ]
    all_robot_center_pos_errors_real = [
        float(row["position_error_robot_center_real_m"]) for row in alignment_rows
    ]
    all_yaw_errors = [float(row["yaw_error_with_fitted_offset_deg"]) for row in alignment_rows]
    summary = {
        "video": str(args.video),
        "report": str(args.report),
        "config": str(args.config),
        "video_meta": video_meta,
        "processed_frames": len(pose_rows),
        "visible_ground_truth_frames": len(valid_rows),
        "alignment_mode": args.alignment_mode,
        "video_log_start_s": args.video_log_start_s,
        "report_log_start_s": float(args.report_log_start_s),
        "segments": alignments,
        "robot_to_marker_offset_m": {
            "x": float(args.robot_to_marker_x_m),
            "y": float(args.robot_to_marker_y_m),
        },
        "marker_to_robot_yaw_offset_deg": args.marker_to_robot_yaw_offset_deg,
        "report_yaw_offset_deg": float(args.report_yaw_offset_deg),
        "report_yaw_offset_by_segment_deg": {
            str(segment): math.degrees(offset)
            for segment, offset in sorted(report_yaw_offset_by_segment_rad.items())
        },
        "report_registration": report_registration,
        "overall_position_error_uncorrected_marker_m": stats(all_pos_errors),
        "overall_position_error_robot_center_with_fitted_yaw_offset_m": stats(all_robot_center_pos_errors),
        "overall_position_error_robot_center_real_m": stats(all_robot_center_pos_errors_real),
        "overall_yaw_error_with_fitted_offset_deg": stats(all_yaw_errors),
        "outputs": {
            "pose_csv": str(pose_csv),
            "alignment_csv": str(alignment_csv),
            "trajectory_svg": str(trajectory_svg),
            "overlay_video": str(overlay_video) if args.write_overlay_video else None,
            "map_video": str(map_video) if args.write_map_video else None,
        },
        "notes": [
            "Position error is computed with the ArUco marker center, not the robot center.",
            "Robot-center position error applies the configured robot-to-marker offset after the marker-to-robot yaw offset.",
            "Yaw error uses the fixed marker-to-robot yaw offset when provided; otherwise it is fitted per segment.",
            "When report_registration is not none, real-map report positions are fitted from aligned video/log samples.",
            "When report_registration is initial_rigid, the report is transformed with rotation+translation from the first aligned pose only.",
            "If the video is a concatenation of clips, each visible segment is aligned independently.",
        ],
    }
    summary_path = output_dir / f"{stem}_alignment_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"pose_csv={pose_csv}")
    print(f"alignment_csv={alignment_csv}")
    print(f"summary_json={summary_path}")
    print(f"trajectory_svg={trajectory_svg}")
    if args.write_overlay_video:
        print(f"overlay_video={overlay_video}")
    if args.write_map_video:
        print(f"map_video={map_video}")
    print(f"processed_frames={len(pose_rows)}")
    print(f"visible_ground_truth_frames={len(valid_rows)}")
    print(f"segments={len(segments)}")
    if all_pos_errors:
        print(f"position_error_mean_m={statistics.fmean(all_pos_errors):.4f}")
        print(f"position_error_p95_m={percentile(all_pos_errors, 95.0):.4f}")
    if all_robot_center_pos_errors:
        print(f"robot_center_position_error_mean_m={statistics.fmean(all_robot_center_pos_errors):.4f}")
        print(f"robot_center_position_error_p95_m={percentile(all_robot_center_pos_errors, 95.0):.4f}")
    if all_robot_center_pos_errors_real:
        print(f"robot_center_real_position_error_mean_m={statistics.fmean(all_robot_center_pos_errors_real):.4f}")
        print(f"robot_center_real_position_error_p95_m={percentile(all_robot_center_pos_errors_real, 95.0):.4f}")
    if all_yaw_errors:
        print(f"yaw_error_mean_deg={statistics.fmean(all_yaw_errors):.2f}")
        print(f"yaw_error_p95_deg={percentile(all_yaw_errors, 95.0):.2f}")
    print(f"report_registration={report_registration.get('mode')}")
    residual = report_registration.get("residual_m")
    if isinstance(residual, dict) and residual.get("mean") is not None:
        print(f"registration_residual_mean_m={float(residual['mean']):.4f}")
        print(f"registration_residual_p95_m={float(residual['p95']):.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
