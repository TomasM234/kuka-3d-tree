from dataclasses import dataclass
import numpy as np

from pose_math import kuka_base_to_matrix, matrix_to_kuka_abc


FEATURE_NAMES = {
    0: "N/A",
    1: "Perimeter",
    2: "External Perimeter",
    3: "Solid Infill",
    4: "Infill",
    5: "Skirt/Brim",
    6: "Support",
}

COLOR_PRINT = np.array([44, 160, 44], dtype=np.uint8)
COLOR_RETRACT = np.array([214, 39, 40], dtype=np.uint8)
COLOR_TRAVEL = np.array([127, 127, 127], dtype=np.uint8)


@dataclass(frozen=True)
class TrajectoryCsvRow:
    move_type: str
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float
    tcp_speed: float
    e_ratio: float
    temp: int
    fan_pct: int
    layer: int
    feature: int
    progress: int


@dataclass(frozen=True)
class TrajectoryData:
    points_xyz: np.ndarray
    orientations_abc: np.ndarray
    colors_rgb: np.ndarray
    layer_end_indices: dict
    max_layer: int
    estimated_time_s: float
    estimated_weight_g: float


def feature_name(feature_id: int, identifier_style: bool = False) -> str:
    name = FEATURE_NAMES.get(feature_id, "Unknown")
    if identifier_style:
        return name.replace(" ", "_").replace("/", "_")
    return name


def segment_color(move_type: str, e_ratio: float):
    if move_type == "P" and e_ratio > 0:
        return COLOR_PRINT
    if move_type in ("R", "U") or (move_type == "P" and e_ratio < 0):
        return COLOR_RETRACT
    return COLOR_TRAVEL


def parse_csv_row(line: str):
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return None

    parts = stripped.split(";")
    if len(parts) >= 14:
        return TrajectoryCsvRow(
            move_type=parts[0],
            x=float(parts[1]),
            y=float(parts[2]),
            z=float(parts[3]),
            a=float(parts[4]),
            b=float(parts[5]),
            c=float(parts[6]),
            tcp_speed=float(parts[7]),
            e_ratio=float(parts[8]),
            temp=int(parts[9]),
            fan_pct=int(parts[10]),
            layer=int(parts[11]),
            feature=int(parts[12]),
            progress=int(parts[13]),
        )

    if len(parts) >= 9:
        return TrajectoryCsvRow(
            move_type=parts[0],
            x=float(parts[1]),
            y=float(parts[2]),
            z=float(parts[3]),
            a=0.0,
            b=0.0,
            c=0.0,
            tcp_speed=float(parts[4]),
            e_ratio=float(parts[5]),
            temp=0,
            fan_pct=0,
            layer=int(parts[8]),
            feature=0,
            progress=0,
        )

    raise ValueError("CSV row must have at least 9 columns.")


def iter_csv_rows(file_obj):
    for line in file_obj:
        row = parse_csv_row(line)
        if row is not None:
            yield row


def load_trajectory_csv(file_path: str) -> TrajectoryData:
    rows = []
    skipped = 0

    with open(file_path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            try:
                row = parse_csv_row(stripped)
            except ValueError:
                skipped += 1
                continue
            if row is not None:
                rows.append(row)

    if not rows:
        raise ValueError(f"No valid data points found in CSV. ({skipped} lines skipped)")

    n = len(rows)
    points_xyz = np.array([(row.x, row.y, row.z) for row in rows], dtype=np.float32)
    orientations_abc = np.array([(row.a, row.b, row.c) for row in rows], dtype=np.float32)
    colors_rgb = np.full((n - 1, 3), COLOR_TRAVEL, dtype=np.uint8)
    for i in range(1, n):
        row = rows[i]
        colors_rgb[i - 1] = segment_color(row.move_type, row.e_ratio)

    layer_ends = {}
    max_layer = 0
    for idx, row in enumerate(rows):
        layer_ends[row.layer] = idx + 1
        if row.layer > max_layer:
            max_layer = row.layer

    estimated_time_s = 0.0
    estimated_weight_g = 0.0
    for i in range(1, n):
        dx = float(points_xyz[i, 0] - points_xyz[i - 1, 0])
        dy = float(points_xyz[i, 1] - points_xyz[i - 1, 1])
        dz = float(points_xyz[i, 2] - points_xyz[i - 1, 2])
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        speed = rows[i].tcp_speed
        e_ratio = rows[i].e_ratio

        if speed > 0.001:
            estimated_time_s += dist / speed
        if e_ratio > 0.0:
            estimated_weight_g += (dist / 1000.0) * e_ratio

    return TrajectoryData(
        points_xyz=points_xyz,
        orientations_abc=orientations_abc,
        colors_rgb=colors_rgb,
        layer_end_indices=layer_ends,
        max_layer=max_layer,
        estimated_time_s=estimated_time_s,
        estimated_weight_g=estimated_weight_g,
    )


def rewrite_trajectory_csv(file_path: str, points_xyz, orientations_abc):
    if points_xyz is None:
        raise ValueError("points_xyz must not be None.")
    points_xyz = np.asarray(points_xyz, dtype=np.float32)
    orientations_abc = None if orientations_abc is None else np.asarray(orientations_abc, dtype=np.float32)
    if orientations_abc is not None and len(points_xyz) != len(orientations_abc):
        raise ValueError("points_xyz and orientations_abc must have equal length.")

    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    point_index = 0
    new_lines = []
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            new_lines.append(line)
            continue

        parts = stripped.split(";")
        if len(parts) >= 4 and point_index < len(points_xyz):
            parts[1] = f"{points_xyz[point_index, 0]:.3f}"
            parts[2] = f"{points_xyz[point_index, 1]:.3f}"
            parts[3] = f"{points_xyz[point_index, 2]:.3f}"
            if orientations_abc is not None and len(parts) >= 14:
                parts[4] = f"{orientations_abc[point_index, 0]:.3f}"
                parts[5] = f"{orientations_abc[point_index, 1]:.3f}"
                parts[6] = f"{orientations_abc[point_index, 2]:.3f}"
            new_lines.append(";".join(parts) + "\n")
            point_index += 1
        else:
            new_lines.append(line)

    if point_index != len(points_xyz):
        raise ValueError(
            f"CSV row count mismatch: expected {len(points_xyz)} writable rows, updated {point_index}."
        )

    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(new_lines)


def apply_planar_edit_transform(points_xyz, orientations_abc, dx, dy, angle_deg):
    if points_xyz is None:
        return None, None

    pts = np.array(points_xyz, dtype=np.float32, copy=True)
    oris = None if orientations_abc is None else np.array(orientations_abc, dtype=np.float32, copy=True)

    if len(pts) == 0:
        return pts, oris

    if angle_deg != 0.0:
        cx = float(np.mean(pts[:, 0]))
        cy = float(np.mean(pts[:, 1]))
        rad = np.radians(angle_deg)
        cos_a = np.cos(rad)
        sin_a = np.sin(rad)

        rel_x = pts[:, 0] - cx
        rel_y = pts[:, 1] - cy
        pts[:, 0] = cos_a * rel_x - sin_a * rel_y + cx
        pts[:, 1] = sin_a * rel_x + cos_a * rel_y + cy

        if oris is not None:
            rot_z = kuka_base_to_matrix(0.0, 0.0, 0.0, angle_deg, 0.0, 0.0)
            for idx, (a, b, c) in enumerate(oris):
                pose = kuka_base_to_matrix(0.0, 0.0, 0.0, float(a), float(b), float(c))
                _, _, _, new_a, new_b, new_c = matrix_to_kuka_abc(rot_z @ pose)
                oris[idx] = (new_a, new_b, new_c)

    pts[:, 0] += dx
    pts[:, 1] += dy

    return pts, oris
