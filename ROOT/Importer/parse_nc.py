"""Importer for NC G-code that uses M3/M5 and S-based extrusion rate."""

import math
import os
import re

from .importer_common import detect_with_matchers, run_importer_cli, write_universal_csv_header


RE_CMD_M3 = re.compile(r"^M3(?:\s|$)")
RE_CMD_M5 = re.compile(r"^M5(?:\s|$)")
RE_STANDALONE_S = re.compile(r"^S([\d.]+)\s*$")


def get_label() -> str:
    return "NC G-code - M3/M5 spindle + S-value extruder"


def get_material_factor() -> float:
    """Machine-specific conversion from S-value into mass flow rate."""
    return 1.0


def can_handle(file_path: str) -> bool:
    """Return True when the file looks like NC spindle-style extrusion."""
    return detect_with_matchers(
        file_path,
        (
            lambda line: RE_CMD_M3.match(line) is not None,
            lambda line: RE_CMD_M5.match(line) is not None,
            lambda line: RE_STANDALONE_S.match(line) is not None,
        ),
    )


def run_import(input_path: str, output_csv: str) -> None:
    """Convert NC G-code to the universal CSV format."""
    factor = get_material_factor()
    layer_height, deposition_width = _read_header_metadata(input_path)
    state = {
        "X": 0.0,
        "Y": 0.0,
        "Z": 0.0,
        "A": 0.0,
        "B": 0.0,
        "C": 0.0,
        "F_Speed": 0.0,
        "S_value": 0.0,
        "extruder_on": False,
        "Temperature": 0,
        "FanPct": 0,
        "LayerIndex": 0,
        "FeatureId": 0,
        "Progress": 0,
        "last_print_z": -999.0,
    }
    rows = []

    try:
        with open(input_path, "r", encoding="utf-8", errors="ignore") as fin:
            for raw_line in fin:
                line = raw_line.strip()
                if not line or line.startswith(";"):
                    continue

                if RE_CMD_M3.match(line):
                    state["extruder_on"] = True
                    continue

                if RE_CMD_M5.match(line):
                    state["extruder_on"] = False
                    continue

                match = RE_STANDALONE_S.match(line)
                if match:
                    state["S_value"] = float(match.group(1))
                    continue

                if line.startswith("M104") or line.startswith("M109"):
                    match = re.search(r"S(\d+)", line)
                    if match:
                        state["Temperature"] = int(match.group(1))
                    continue

                if line.startswith("M106"):
                    match = re.search(r"S(\d+)", line)
                    if match:
                        state["FanPct"] = int(int(match.group(1)) * 100 / 255)
                    continue

                if line.startswith("M107"):
                    state["FanPct"] = 0
                    continue

                if line.startswith("G92"):
                    continue

                if line.startswith("G0") or line.startswith("G1"):
                    clean_line = line.split(";", 1)[0].strip()
                    row = _parse_move(clean_line, state, factor)
                    if row is not None:
                        rows.append(row)

    except FileNotFoundError as exc:
        raise RuntimeError(f"Input file not found: {input_path}") from exc
    except OSError as exc:
        raise RuntimeError(f"I/O error: {exc}") from exc

    if not rows:
        raise RuntimeError("No valid moves found - wrong file format?")

    rows = _with_progress(rows)

    header_info = f"Source: {os.path.basename(input_path)}"
    if layer_height:
        header_info += f" | Layer height: {layer_height}"
    if deposition_width:
        header_info += f" | Deposition width: {deposition_width}"

    try:
        with open(output_csv, "w", encoding="utf-8") as fout:
            write_universal_csv_header(fout, "NC G-code Importer", header_info)
            for row in rows:
                fout.write(row + "\n")
    except OSError as exc:
        raise RuntimeError(f"Cannot write output file: {exc}") from exc


def _read_header_metadata(input_path: str):
    layer_height = ""
    deposition_width = ""
    try:
        with open(input_path, "r", encoding="utf-8", errors="ignore") as file_obj:
            for raw_line in file_obj:
                line = raw_line.strip()
                if not line.startswith(";"):
                    if line.startswith(("G", "M", "S")):
                        break
                    continue
                match = re.search(r"Layer height:\s*([\d.]+)", line)
                if match:
                    layer_height = match.group(1)
                match = re.search(r"Deposition width:\s*([\d.]+)", line)
                if match:
                    deposition_width = match.group(1)
    except OSError:
        pass
    return layer_height, deposition_width


def _with_progress(rows):
    total = len(rows)
    if total == 1:
        return [f"{rows[0].rsplit(';', 1)[0]};100"]
    return [
        f"{row.rsplit(';', 1)[0]};{int(round(index * 100 / (total - 1)))}"
        for index, row in enumerate(rows)
    ]


def _parse_move(line: str, state: dict, factor: float):
    parts = line.split()
    cmd = parts[0]

    old_x, old_y, old_z = state["X"], state["Y"], state["Z"]
    old_a, old_b, old_c = state["A"], state["B"], state["C"]

    for param in parts[1:]:
        key = param[0]
        try:
            value = float(param[1:].replace("=", ""))
        except ValueError:
            continue

        if key == "X":
            state["X"] = value
        elif key == "Y":
            state["Y"] = value
        elif key == "Z":
            state["Z"] = value
        elif key == "A":
            state["A"] = value
        elif key == "B":
            state["B"] = value
        elif key == "C":
            state["C"] = value
        elif key == "F":
            state["F_Speed"] = value

    linear_dist = math.sqrt(
        (state["X"] - old_x) ** 2
        + (state["Y"] - old_y) ** 2
        + (state["Z"] - old_z) ** 2
    )
    orientation_delta = max(
        abs(state["A"] - old_a),
        abs(state["B"] - old_b),
        abs(state["C"] - old_c),
    )
    has_linear_motion = linear_dist >= 0.001
    has_orientation_motion = orientation_delta >= 0.001
    if not has_linear_motion and not has_orientation_motion:
        return None

    if cmd == "G0":
        action = "T"
    elif not has_linear_motion:
        action = "T"
    elif cmd == "G1" and state["extruder_on"]:
        action = "P"
    else:
        action = "T"

    if action == "P":
        if state["last_print_z"] == -999.0:
            state["last_print_z"] = state["Z"]
        elif abs(state["Z"] - state["last_print_z"]) > 0.01:
            state["LayerIndex"] += 1
            state["last_print_z"] = state["Z"]

    tcp_speed = state["F_Speed"] / 60.0
    if action == "P" and tcp_speed > 0.001:
        e_ratio = state["S_value"] * factor / tcp_speed
    else:
        e_ratio = 0.0

    return (
        f"{action};{state['X']:.3f};{state['Y']:.3f};{state['Z']:.3f};"
        f"{state['A']:.3f};{state['B']:.3f};{state['C']:.3f};"
        f"{tcp_speed:.3f};{e_ratio:.6f};"
        f"{state['Temperature']};{state['FanPct']};"
        f"{state['LayerIndex']};{state['FeatureId']};{state['Progress']}"
    )


if __name__ == "__main__":
    run_importer_cli(get_label(), run_import, get_material_factor)
