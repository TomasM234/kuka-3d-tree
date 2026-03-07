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


def feature_name(feature_id: int, identifier_style: bool = False) -> str:
    name = FEATURE_NAMES.get(feature_id, "Unknown")
    if identifier_style:
        return name.replace(" ", "_").replace("/", "_")
    return name


def parse_csv_row(line: str):
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return None

    parts = stripped.split(";")
    if len(parts) < 14:
        raise ValueError("CSV row must have at least 14 columns.")

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


def iter_csv_rows(file_obj):
    for line in file_obj:
        row = parse_csv_row(line)
        if row is not None:
            yield row


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
