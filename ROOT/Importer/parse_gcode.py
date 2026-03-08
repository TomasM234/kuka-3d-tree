"""Importer for slicer G-code that uses the E axis for extrusion."""

import math
import re

from .importer_common import detect_with_matchers, run_importer_cli, write_universal_csv_header


def get_label() -> str:
    return "PrusaSlicer / FDM Slicer G-code (E-axis)"


def get_material_factor() -> float:
    """
    Convert raw E/distance ratio into grams per meter.

    The default value matches a typical 1.75 mm PLA setup.
    """
    return 3.006


def can_handle(file_path: str) -> bool:
    """Return True when the file looks like slicer G-code."""
    return detect_with_matchers(
        file_path,
        (
            lambda line: line.startswith("G1") and " E" in line,
            lambda line: line.startswith(";TYPE:"),
            lambda line: line.startswith(";LAYER_CHANGE"),
        ),
    )


def run_import(input_path: str, output_csv: str) -> None:
    """Convert a slicer G-code file to the universal CSV format."""
    factor = get_material_factor()
    state = {
        "X": 0.0,
        "Y": 0.0,
        "Z": 0.0,
        "A": 0.0,
        "B": 0.0,
        "C": 0.0,
        "E": 0.0,
        "previous_E": 0.0,
        "F_Speed": 0.0,
        "Temperature": 0,
        "FanPwm": 0,
        "LayerIndex": 0,
        "LayerChangeCount": 0,
        "FeatureId": 0,
        "Progress": 0,
    }
    row_count = 0

    try:
        with open(input_path, "r", encoding="utf-8", errors="ignore") as fin, open(
            output_csv, "w", encoding="utf-8"
        ) as fout:
            write_universal_csv_header(fout, "PrusaSlicer G-code Importer")

            for raw_line in fin:
                line = raw_line.strip()
                if not line:
                    continue

                if line.startswith("G92"):
                    match = re.search(r"\bE(-?\d+(?:\.\d+)?)", line)
                    if match:
                        state["E"] = float(match.group(1))
                        state["previous_E"] = state["E"]
                    continue

                if line.startswith("M104") or line.startswith("M109"):
                    match = re.search(r"S(\d+)", line)
                    if match:
                        state["Temperature"] = int(match.group(1))
                    continue

                if line.startswith("M106"):
                    match = re.search(r"S(\d+)", line)
                    if match:
                        state["FanPwm"] = int(match.group(1))
                    continue

                if line.startswith("M107"):
                    state["FanPwm"] = 0
                    continue

                if line.startswith("M73"):
                    match = re.search(r"P(\d+)", line)
                    if match:
                        state["Progress"] = int(match.group(1))
                    continue

                if line.startswith(";LAYER_CHANGE"):
                    state["LayerChangeCount"] += 1
                    state["LayerIndex"] = max(0, state["LayerChangeCount"] - 1)
                    continue

                if line.startswith(";TYPE:"):
                    state["FeatureId"] = _parse_feature_type(line.split(";TYPE:", 1)[1])
                    continue

                if line.startswith("G1"):
                    clean_line = line.split(";", 1)[0].strip()
                    row = _parse_g1(clean_line, state, factor)
                    if row is not None:
                        fout.write(row + "\n")
                        row_count += 1

    except FileNotFoundError as exc:
        raise RuntimeError(f"Input file not found: {input_path}") from exc
    except OSError as exc:
        raise RuntimeError(f"I/O error: {exc}") from exc

    if row_count == 0:
        raise RuntimeError("No valid G1 moves found - wrong file format?")


def _parse_feature_type(type_string: str) -> int:
    lowered = type_string.lower().strip()
    if "external perimeter" in lowered:
        return 2
    if "perimeter" in lowered:
        return 1
    if "solid infill" in lowered or "top solid infill" in lowered:
        return 3
    if "infill" in lowered:
        return 4
    if "skirt" in lowered or "brim" in lowered:
        return 5
    if "support" in lowered:
        return 6
    return 0


def _parse_g1(line: str, state: dict, factor: float):
    old_x, old_y, old_z = state["X"], state["Y"], state["Z"]

    for param in line.split()[1:]:
        if param.startswith("X"):
            state["X"] = float(param[1:])
        elif param.startswith("Y"):
            state["Y"] = float(param[1:])
        elif param.startswith("Z"):
            state["Z"] = float(param[1:])
        elif param.startswith("F"):
            state["F_Speed"] = float(param[1:])
        elif param.startswith("E"):
            state["E"] = float(param[1:])

    dist = math.sqrt(
        (state["X"] - old_x) ** 2
        + (state["Y"] - old_y) ** 2
        + (state["Z"] - old_z) ** 2
    )
    delta_e = state["E"] - state["previous_E"]
    state["previous_E"] = state["E"]

    if dist > 0.001 and delta_e > 0:
        action = "P"
    elif dist > 0.001:
        action = "T"
    elif delta_e < 0:
        action = "R"
    elif delta_e > 0:
        action = "U"
    else:
        return None

    e_ratio = (delta_e / dist * factor) if action == "P" else 0.0
    tcp_speed = state["F_Speed"] / 60.0
    fan_pct = int(state["FanPwm"] * 100 / 255)

    return (
        f"{action};{state['X']:.3f};{state['Y']:.3f};{state['Z']:.3f};"
        f"{state['A']:.3f};{state['B']:.3f};{state['C']:.3f};"
        f"{tcp_speed:.3f};{e_ratio:.6f};"
        f"{state['Temperature']};{fan_pct};"
        f"{state['LayerIndex']};{state['FeatureId']};{state['Progress']}"
    )


if __name__ == "__main__":
    run_importer_cli(get_label(), run_import, get_material_factor)
