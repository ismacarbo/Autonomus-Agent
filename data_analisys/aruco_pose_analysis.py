"""Detect ArUco/AprilTag markers and estimate planar poses.

The script has two useful modes:

1. Detection only: report marker ids, image centers, pixel yaw and corners.
2. Planar pose: if a JSON config maps fixed marker ids to map coordinates,
   estimate an image-to-map homography and export metric marker poses.

Example:
    python -m data_analisys.aruco_pose_analysis \
        --image aruco_test.jpg \
        --config aruco_ground_truth_config.json \
        --robot-id 12 \
        --output-dir data_analisys/outputs/aruco_pose_test
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

cv2 = None
np = None


DEFAULT_DICTIONARIES = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_7X7_50",
    "DICT_APRILTAG_16h5",
    "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10",
    "DICT_APRILTAG_36h11",
]


def require_opencv() -> None:
    global cv2, np
    if cv2 is not None and np is not None:
        return
    try:
        import numpy as numpy_module
        import cv2 as cv2_module
    except ModuleNotFoundError as exc:  # pragma: no cover - depends on local env
        raise SystemExit(
            "OpenCV/NumPy non sono installati. Installa il modulo contrib con:\n"
            "  python -m pip install opencv-contrib-python-headless numpy\n"
        ) from exc
    cv2 = cv2_module
    np = numpy_module


@dataclass
class MarkerDetection:
    marker_id: int
    dictionary: str
    corners_px: np.ndarray

    @property
    def center_px(self) -> np.ndarray:
        return self.corners_px.mean(axis=0)

    @property
    def yaw_px_rad(self) -> float:
        # OpenCV orders marker corners clockwise in marker coordinates:
        # top-left, top-right, bottom-right, bottom-left.
        axis = self.corners_px[1] - self.corners_px[0]
        return math.atan2(float(axis[1]), float(axis[0]))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_deg(angle: float) -> float:
    return math.degrees(wrap_angle(angle))


def load_config(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def reference_box_config(
    width_m: float,
    height_m: float,
    id_a: int,
    id_b: int,
    id_c: int,
    id_d: int,
) -> dict[str, Any]:
    if width_m <= 0.0 or height_m <= 0.0:
        raise SystemExit("--reference-width-m e --reference-height-m devono essere positivi")
    return {
        "reference_box": {
            "convention": "C=(0,0), B=(W,0), A=(W,H), D=(0,H)",
            "width_m": width_m,
            "height_m": height_m,
            "ids": {"A": id_a, "B": id_b, "C": id_c, "D": id_d},
        },
        "reference_markers": {
            str(id_c): {"center": [0.0, 0.0]},
            str(id_b): {"center": [width_m, 0.0]},
            str(id_a): {"center": [width_m, height_m]},
            str(id_d): {"center": [0.0, height_m]},
        },
    }


def dictionary_names_from_config(config: dict[str, Any]) -> list[str]:
    names = config.get("dictionaries")
    if isinstance(names, list) and names:
        return [str(name) for name in names]
    name = config.get("dictionary")
    if isinstance(name, str) and name:
        return [name]
    return DEFAULT_DICTIONARIES


def make_detector(dictionary_name: str):
    require_opencv()
    aruco = cv2.aruco
    dictionary_id = getattr(aruco, dictionary_name, None)
    if dictionary_id is None:
        return None
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(aruco, "DetectorParameters"):
        parameters = aruco.DetectorParameters()
    else:
        parameters = aruco.DetectorParameters_create()
    if hasattr(parameters, "cornerRefinementMethod") and hasattr(aruco, "CORNER_REFINE_SUBPIX"):
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    if hasattr(aruco, "ArucoDetector"):
        return aruco.ArucoDetector(dictionary, parameters)
    return dictionary, parameters


def detect_markers(image: np.ndarray, dictionary_names: list[str]) -> list[MarkerDetection]:
    require_opencv()
    if not hasattr(cv2, "aruco"):
        raise SystemExit(
            "Il modulo cv2.aruco non e' disponibile. Serve opencv-contrib-python, "
            "non il pacchetto opencv-python base."
        )

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections_by_id: dict[int, MarkerDetection] = {}
    for dictionary_name in dictionary_names:
        detector = make_detector(dictionary_name)
        if detector is None:
            continue
        if hasattr(cv2.aruco, "ArucoDetector") and hasattr(detector, "detectMarkers"):
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            dictionary, parameters = detector
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        if ids is None:
            continue
        for corner_set, marker_id_arr in zip(corners, ids.flatten()):
            marker_id = int(marker_id_arr)
            marker_corners = np.asarray(corner_set, dtype=np.float64).reshape(4, 2)
            previous = detections_by_id.get(marker_id)
            if previous is None:
                detections_by_id[marker_id] = MarkerDetection(
                    marker_id=marker_id,
                    dictionary=dictionary_name,
                    corners_px=marker_corners,
                )
    return sorted(detections_by_id.values(), key=lambda item: item.marker_id)


def marker_world_points(marker_cfg: dict[str, Any]) -> np.ndarray:
    require_opencv()
    if "corners" in marker_cfg:
        return np.asarray(marker_cfg["corners"], dtype=np.float64).reshape(4, 2)
    center = np.asarray(marker_cfg["center"], dtype=np.float64)
    size_m = float(marker_cfg["size_m"])
    yaw = float(marker_cfg.get("yaw_rad", marker_cfg.get("yaw", 0.0)))
    half = 0.5 * size_m
    local = np.array(
        [
            [-half, -half],
            [half, -half],
            [half, half],
            [-half, half],
        ],
        dtype=np.float64,
    )
    rotation = np.array(
        [
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ],
        dtype=np.float64,
    )
    return center + local @ rotation.T


def estimate_homography(
    detections: list[MarkerDetection],
    config: dict[str, Any],
) -> tuple[np.ndarray | None, list[int], float | None]:
    require_opencv()
    refs = config.get("reference_markers", {})
    if not isinstance(refs, dict) or not refs:
        return None, [], None

    detected_by_id = {det.marker_id: det for det in detections}
    image_points: list[np.ndarray] = []
    world_points: list[np.ndarray] = []
    used_ids: list[int] = []
    for id_text, marker_cfg in refs.items():
        marker_id = int(id_text)
        det = detected_by_id.get(marker_id)
        if det is None:
            continue
        if "size_m" in marker_cfg or "corners" in marker_cfg:
            image_points.append(det.corners_px)
            world_points.append(marker_world_points(marker_cfg))
        else:
            image_points.append(det.center_px.reshape(1, 2))
            world_points.append(np.asarray(marker_cfg["center"], dtype=np.float64).reshape(1, 2))
        used_ids.append(marker_id)

    if sum(points.shape[0] for points in image_points) < 4:
        return None, used_ids, None

    image_array = np.vstack(image_points).astype(np.float64)
    world_array = np.vstack(world_points).astype(np.float64)
    homography, mask = cv2.findHomography(image_array, world_array, method=cv2.RANSAC, ransacReprojThreshold=0.015)
    if homography is None:
        return None, used_ids, None

    projected = apply_homography(homography, image_array)
    reprojection_rmse = float(np.sqrt(np.mean(np.sum((projected - world_array) ** 2, axis=1))))
    return homography, used_ids, reprojection_rmse


def apply_homography(homography: np.ndarray, points_px: np.ndarray) -> np.ndarray:
    require_opencv()
    points = np.asarray(points_px, dtype=np.float64).reshape(-1, 2)
    homogeneous = np.column_stack([points, np.ones(points.shape[0])])
    transformed = homogeneous @ homography.T
    transformed /= transformed[:, 2:3]
    return transformed[:, :2]


def marker_pose_from_homography(det: MarkerDetection, homography: np.ndarray) -> dict[str, Any]:
    corners_m = apply_homography(homography, det.corners_px)
    center_m = corners_m.mean(axis=0)
    axis = corners_m[1] - corners_m[0]
    yaw_m = math.atan2(float(axis[1]), float(axis[0]))
    return {
        "id": det.marker_id,
        "dictionary": det.dictionary,
        "center_px": det.center_px.tolist(),
        "yaw_px_rad": det.yaw_px_rad,
        "yaw_px_deg": yaw_to_deg(det.yaw_px_rad),
        "corners_px": det.corners_px.tolist(),
        "center_m": center_m.tolist(),
        "yaw_m_rad": yaw_m,
        "yaw_m_deg": yaw_to_deg(yaw_m),
        "corners_m": corners_m.tolist(),
    }


def marker_pose_pixels(det: MarkerDetection) -> dict[str, Any]:
    return {
        "id": det.marker_id,
        "dictionary": det.dictionary,
        "center_px": det.center_px.tolist(),
        "yaw_px_rad": det.yaw_px_rad,
        "yaw_px_deg": yaw_to_deg(det.yaw_px_rad),
        "corners_px": det.corners_px.tolist(),
    }


def draw_overlay(
    image: np.ndarray,
    detections: list[MarkerDetection],
    poses: list[dict[str, Any]],
    robot_id: int | None,
) -> np.ndarray:
    require_opencv()
    annotated = image.copy()
    pose_by_id = {int(pose["id"]): pose for pose in poses}
    for det in detections:
        corners = det.corners_px.astype(np.int32)
        color = (0, 190, 255) if det.marker_id == robot_id else (80, 220, 80)
        cv2.polylines(annotated, [corners], True, color, 3)
        center = tuple(np.round(det.center_px).astype(int))
        axis_end = tuple(np.round(det.center_px + 0.65 * (det.corners_px[1] - det.corners_px[0])).astype(int))
        cv2.arrowedLine(annotated, center, axis_end, (40, 80, 255), 3, tipLength=0.25)
        pose = pose_by_id.get(det.marker_id, {})
        if "center_m" in pose:
            label = f"id {det.marker_id} ({pose['center_m'][0]:.3f},{pose['center_m'][1]:.3f}) {pose['yaw_m_deg']:.1f}deg"
        else:
            label = f"id {det.marker_id} {det.dictionary}"
        cv2.putText(
            annotated,
            label,
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return annotated


def map_bounds_from_result(result: dict[str, Any]) -> tuple[float, float, float, float] | None:
    reference_box = result.get("reference_box")
    if isinstance(reference_box, dict):
        width = reference_box.get("width_m")
        height = reference_box.get("height_m")
        if isinstance(width, (int, float)) and isinstance(height, (int, float)):
            if float(width) > 0.0 and float(height) > 0.0:
                return 0.0, float(width), 0.0, float(height)

    points = [
        pose["center_m"]
        for pose in result.get("markers", [])
        if isinstance(pose.get("center_m"), list) and len(pose["center_m"]) == 2
    ]
    if not points:
        return None
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    margin = max(span_x, span_y, 0.1) * 0.08
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def world_to_canvas_matrix(
    min_x: float,
    min_y: float,
    pixels_per_meter: float,
    padding_px: int,
) -> np.ndarray:
    require_opencv()
    return np.array(
        [
            [pixels_per_meter, 0.0, padding_px - pixels_per_meter * min_x],
            [0.0, pixels_per_meter, padding_px - pixels_per_meter * min_y],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def world_point_to_canvas(
    point_m: list[float] | tuple[float, float] | np.ndarray,
    transform: np.ndarray,
) -> tuple[int, int]:
    require_opencv()
    point = np.asarray([float(point_m[0]), float(point_m[1]), 1.0], dtype=np.float64)
    mapped = transform @ point
    mapped /= mapped[2]
    return int(round(mapped[0])), int(round(mapped[1]))


def draw_map_grid(
    canvas: np.ndarray,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    transform: np.ndarray,
) -> None:
    require_opencv()
    width_m = max_x - min_x
    height_m = max_y - min_y
    grid_step = 0.10
    if max(width_m, height_m) > 2.5:
        grid_step = 0.25
    if max(width_m, height_m) > 5.0:
        grid_step = 0.50

    start_x = math.floor(min_x / grid_step) * grid_step
    end_x = math.ceil(max_x / grid_step) * grid_step
    x = start_x
    while x <= end_x + 1e-9:
        p0 = world_point_to_canvas((x, min_y), transform)
        p1 = world_point_to_canvas((x, max_y), transform)
        color = (225, 225, 225) if abs(x) > 1e-9 else (200, 200, 200)
        cv2.line(canvas, p0, p1, color, 1, cv2.LINE_AA)
        x += grid_step

    start_y = math.floor(min_y / grid_step) * grid_step
    end_y = math.ceil(max_y / grid_step) * grid_step
    y = start_y
    while y <= end_y + 1e-9:
        p0 = world_point_to_canvas((min_x, y), transform)
        p1 = world_point_to_canvas((max_x, y), transform)
        color = (225, 225, 225) if abs(y) > 1e-9 else (200, 200, 200)
        cv2.line(canvas, p0, p1, color, 1, cv2.LINE_AA)
        y += grid_step

    top_left = world_point_to_canvas((min_x, min_y), transform)
    bottom_right = world_point_to_canvas((max_x, max_y), transform)
    cv2.rectangle(canvas, top_left, bottom_right, (70, 70, 70), 2, cv2.LINE_AA)


def draw_map_view(
    image: np.ndarray,
    result: dict[str, Any],
    pixels_per_meter: float,
    padding_px: int,
) -> np.ndarray | None:
    require_opencv()
    homography_data = result.get("homography", {})
    if not homography_data.get("available"):
        return None
    image_to_map = homography_data.get("image_to_map")
    if image_to_map is None:
        return None
    bounds = map_bounds_from_result(result)
    if bounds is None:
        return None

    min_x, max_x, min_y, max_y = bounds
    width_px = int(math.ceil((max_x - min_x) * pixels_per_meter)) + 2 * padding_px
    height_px = int(math.ceil((max_y - min_y) * pixels_per_meter)) + 2 * padding_px
    width_px = max(width_px, 320)
    height_px = max(height_px, 320)

    world_to_canvas = world_to_canvas_matrix(min_x, min_y, pixels_per_meter, padding_px)
    image_to_canvas = world_to_canvas @ np.asarray(image_to_map, dtype=np.float64)
    warped = cv2.warpPerspective(
        image,
        image_to_canvas,
        (width_px, height_px),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(248, 248, 248),
    )

    grid = np.full_like(warped, 248)
    draw_map_grid(grid, min_x, max_x, min_y, max_y, world_to_canvas)
    map_view = cv2.addWeighted(grid, 0.35, warped, 0.65, 0.0)
    draw_map_grid(map_view, min_x, max_x, min_y, max_y, world_to_canvas)

    robot_id = result.get("robot_id")
    reference_ids = set(homography_data.get("reference_ids", []))
    label_by_id = {}
    reference_box = result.get("reference_box")
    if isinstance(reference_box, dict) and isinstance(reference_box.get("ids"), dict):
        label_by_id = {int(value): str(key) for key, value in reference_box["ids"].items()}

    for pose in result.get("markers", []):
        center_m = pose.get("center_m")
        if not isinstance(center_m, list) or len(center_m) != 2:
            continue
        center = world_point_to_canvas(center_m, world_to_canvas)
        marker_id = int(pose["id"])
        is_robot = robot_id is not None and marker_id == int(robot_id)
        color = (0, 190, 255) if is_robot else (60, 180, 70)
        if marker_id in reference_ids:
            color = (55, 155, 55)
        cv2.circle(map_view, center, 7, color, -1, cv2.LINE_AA)

        yaw_rad = float(pose.get("yaw_m_rad", pose.get("yaw_px_rad", 0.0)))
        arrow_m = 0.12 * max(max_x - min_x, max_y - min_y, 0.5)
        arrow_tip_m = [
            float(center_m[0]) + math.cos(yaw_rad) * arrow_m,
            float(center_m[1]) + math.sin(yaw_rad) * arrow_m,
        ]
        arrow_tip = world_point_to_canvas(arrow_tip_m, world_to_canvas)
        cv2.arrowedLine(map_view, center, arrow_tip, (40, 80, 255), 2, cv2.LINE_AA, tipLength=0.25)

        label_prefix = label_by_id.get(marker_id, f"id {marker_id}")
        label = f"{label_prefix} ({center_m[0]:.3f},{center_m[1]:.3f})"
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 2)
        label_x = center[0] + 8
        label_y = center[1] - 8
        if label_x + text_size[0] > map_view.shape[1] - 8:
            label_x = center[0] - text_size[0] - 8
        if label_y - text_size[1] < 36:
            label_y = center[1] + text_size[1] + 12
        cv2.putText(
            map_view,
            label,
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            color,
            2,
            cv2.LINE_AA,
        )

    header = f"Map frame: x right, y down | scale {pixels_per_meter:.0f} px/m"
    cv2.putText(
        map_view,
        header,
        (14, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (30, 30, 30),
        2,
        cv2.LINE_AA,
    )
    return map_view


def write_outputs(
    output_dir: Path,
    image_path: Path,
    annotated: np.ndarray,
    result: dict[str, Any],
    map_view: np.ndarray | None = None,
) -> None:
    require_opencv()
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = image_path.stem
    json_path = output_dir / f"{stem}_aruco_pose.json"
    csv_path = output_dir / f"{stem}_aruco_pose.csv"
    image_out = output_dir / f"{stem}_aruco_overlay.png"
    map_out = output_dir / f"{stem}_aruco_map.png"

    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2)

    poses = result["markers"]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "id",
            "dictionary",
            "center_px_x",
            "center_px_y",
            "yaw_px_deg",
            "center_m_x",
            "center_m_y",
            "yaw_m_deg",
            "role",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        robot_id = result.get("robot_id")
        reference_ids = set(result.get("homography", {}).get("reference_ids", []))
        for pose in poses:
            center_px = pose["center_px"]
            center_m = pose.get("center_m", [None, None])
            role = "robot" if pose["id"] == robot_id else ("reference" if pose["id"] in reference_ids else "marker")
            writer.writerow(
                {
                    "id": pose["id"],
                    "dictionary": pose["dictionary"],
                    "center_px_x": center_px[0],
                    "center_px_y": center_px[1],
                    "yaw_px_deg": pose["yaw_px_deg"],
                    "center_m_x": center_m[0],
                    "center_m_y": center_m[1],
                    "yaw_m_deg": pose.get("yaw_m_deg"),
                    "role": role,
                }
            )

    cv2.imwrite(str(image_out), annotated)
    if map_view is not None:
        cv2.imwrite(str(map_out), map_view)
    print(f"json={json_path}")
    print(f"csv={csv_path}")
    print(f"overlay={image_out}")
    if map_view is not None:
        print(f"map={map_out}")


def write_sample_config(path: Path) -> None:
    sample = {
        "dictionary": "DICT_4X4_50",
        "robot_id": 99,
        "reference_box": {
            "convention": "C=(0,0), B=(W,0), A=(W,H), D=(0,H)",
            "width_m": 1.00,
            "height_m": 1.00,
            "ids": {"A": 1, "B": 2, "C": 3, "D": 4},
        },
        "reference_markers": {
            "3": {"center": [0.00, 0.00]},
            "2": {"center": [1.00, 0.00]},
            "1": {"center": [1.00, 1.00]},
            "4": {"center": [0.00, 1.00]},
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(sample, handle, indent=2)
    print(f"sample_config={path}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, help="Foto/frame da analizzare")
    parser.add_argument("--config", type=Path, help="JSON con marker reference e coordinate reali")
    parser.add_argument("--robot-id", type=int, help="ID del marker sul robot")
    parser.add_argument("--output-dir", type=Path, default=Path("data_analisys/outputs/aruco_pose"))
    parser.add_argument("--dictionary", action="append", help="Dizionario OpenCV da provare; ripetibile")
    parser.add_argument(
        "--reference-width-m",
        "--box-width-m",
        type=float,
        help="W reale tra i reference sinistri e destri: C/B e D/A",
    )
    parser.add_argument(
        "--reference-height-m",
        "--box-height-m",
        type=float,
        help="H reale tra i reference alti e bassi: C/D e B/A",
    )
    parser.add_argument("--reference-id-a", type=int, default=1, help="ID reference A, basso destra")
    parser.add_argument("--reference-id-b", type=int, default=2, help="ID reference B, alto destra")
    parser.add_argument("--reference-id-c", type=int, default=3, help="ID reference C, alto sinistra")
    parser.add_argument("--reference-id-d", type=int, default=4, help="ID reference D, basso sinistra")
    parser.add_argument("--map-pixels-per-meter", type=float, default=650.0, help="Scala PNG mappa rettificata")
    parser.add_argument("--map-padding-px", type=int, default=70, help="Margine PNG mappa rettificata")
    parser.add_argument("--write-sample-config", type=Path, help="Scrive un config JSON di esempio ed esce")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    if args.write_sample_config is not None:
        write_sample_config(args.write_sample_config)
        return 0
    if args.image is None:
        raise SystemExit("Serve --image oppure --write-sample-config")

    require_opencv()
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"Impossibile leggere immagine: {args.image}")

    config = load_config(args.config)
    if args.reference_width_m is not None or args.reference_height_m is not None:
        if args.reference_width_m is None or args.reference_height_m is None:
            raise SystemExit("Usa insieme --reference-width-m e --reference-height-m")
        box_config = reference_box_config(
            args.reference_width_m,
            args.reference_height_m,
            args.reference_id_a,
            args.reference_id_b,
            args.reference_id_c,
            args.reference_id_d,
        )
        config = {**config, **box_config}
    robot_id = args.robot_id if args.robot_id is not None else config.get("robot_id")
    dictionaries = args.dictionary or dictionary_names_from_config(config)
    detections = detect_markers(image, dictionaries)
    homography, used_reference_ids, reprojection_rmse = estimate_homography(detections, config)

    if homography is not None:
        poses = [marker_pose_from_homography(det, homography) for det in detections]
    else:
        poses = [marker_pose_pixels(det) for det in detections]

    result = {
        "image": str(args.image),
        "image_size": {"width": int(image.shape[1]), "height": int(image.shape[0])},
        "dictionaries": dictionaries,
        "robot_id": robot_id,
        "markers": poses,
        "reference_box": config.get("reference_box"),
        "homography": {
            "available": homography is not None,
            "reference_ids": used_reference_ids,
            "reprojection_rmse_m": reprojection_rmse,
            "image_to_map": homography.tolist() if homography is not None else None,
        },
    }

    annotated = draw_overlay(image, detections, poses, int(robot_id) if robot_id is not None else None)
    map_view = draw_map_view(
        image,
        result,
        max(args.map_pixels_per_meter, 1.0),
        max(args.map_padding_px, 0),
    )
    write_outputs(args.output_dir, args.image, annotated, result, map_view)

    print(f"detected_markers={len(detections)}")
    if homography is None and config.get("reference_markers"):
        print(
            "homography_warning=servono almeno 4 marker reference rilevati e mappati "
            f"(rilevati nel config: {used_reference_ids})"
        )
    if robot_id is not None:
        robot_pose = next((pose for pose in poses if int(pose["id"]) == int(robot_id)), None)
        if robot_pose is not None:
            if "center_m" in robot_pose:
                print(
                    "robot_pose="
                    f"x={robot_pose['center_m'][0]:.4f} "
                    f"y={robot_pose['center_m'][1]:.4f} "
                    f"yaw_deg={robot_pose['yaw_m_deg']:.2f}"
                )
            else:
                print(
                    "robot_pose_px="
                    f"x={robot_pose['center_px'][0]:.1f} "
                    f"y={robot_pose['center_px'][1]:.1f} "
                    f"yaw_deg={robot_pose['yaw_px_deg']:.2f}"
                )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
