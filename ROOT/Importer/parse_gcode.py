"""
Importer: PrusaSlicer / FDM Slicer G-code (E-axis extrusion)
=============================================================

EXPECTED INPUT FORMAT
---------------------
Standard FDM slicer G-code, as exported by PrusaSlicer, SuperSlicer,
Bambu Studio, Cura, or any slicer that uses the E axis for extrusion.

Key commands interpreted:
  G1 [X..] [Y..] [Z..] [F..] [E..]
      Linear move. X/Y/Z set position [mm], F sets feedrate [mm/min],
      E is the extruder position [mm of filament].
  G92 E0
      Reset extruder position to 0 (used before/after retract).
  M104 S<temp> / M109 S<temp>
      Set / wait for hotend temperature [°C].
  M106 S<0-255>
      Part-cooling fan speed (0 = off, 255 = 100 %).
  M107
      Fan off.
  M73 P<0-100>
      Print progress percentage (injected by slicer).
  ;LAYER_CHANGE
      Comment injected by PrusaSlicer to mark layer transitions.
  ;TYPE:<name>
      Comment marking the current feature type (Perimeter, Infill, …).

Move classification (same segment can only be one):
  P  Print   — XY/Z movement AND positive delta-E (extruding)
  T  Travel  — XY/Z movement, zero or negative delta-E (no extrusion)
  R  Retract — no significant XY/Z movement, negative delta-E
  U  Unretract — no significant XY/Z movement, positive delta-E

OUTPUT CSV FORMAT
-----------------
  # comment lines (ignored by viewer)
  TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS

  TYPE      P / T / R / U  (see above)
  X/Y/Z     TCP position [mm]
  A/B/C     TCP orientation [deg]  — always 0.0 from this importer
  TCP_SPEED feedrate [mm/s]
  E_RATIO   [mm_filament / mm_path] × material_factor  (0.0 for non-print)
  TEMP      last known hotend setpoint [°C]
  FAN_PCT   fan speed [0–100]
  LAYER     layer index (0-based)
  FEATURE   integer ID: 0=N/A 1=Perimeter 2=ExtPerimeter 3=SolidInfill
                        4=Infill 5=Skirt/Brim 6=Support
  PROGRESS  print progress [0–100 %]
"""

import argparse
import math
import os
import re
import sys


# ---------------------------------------------------------------------------
# Plugin interface — edit these two values when adapting for a new slicer
# ---------------------------------------------------------------------------

def get_label() -> str:
    """Short human-readable name shown in the viewer's importer combobox."""
    return "PrusaSlicer / FDM Slicer G-code (E-axis)"


def get_material_factor() -> float:
    """
    Multiplier applied to raw E_RATIO before writing to CSV.

    E_RATIO (raw) = mm_filament_extruded / mm_path_length

    For 1.75 mm filament the volumetric equivalent is:
        volumetric [mm³/mm] = raw_ratio × π × (d/2)²  ≈ raw_ratio × 2.405

    Set to 1.0 to keep the raw filament-mm unit (default).
    Adjust if your postprocessor or PLC expects a different unit.
    """
    return 1.0


# ---------------------------------------------------------------------------
# Plugin interface — detection and conversion
# ---------------------------------------------------------------------------

def can_handle(file_path: str) -> bool:
    """
    Return True if this file looks like slicer G-code with E-axis extrusion.
    Scans only the first 200 lines for speed.
    """
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for i, line in enumerate(f):
                if i >= 200:
                    break
                s = line.strip()
                if s.startswith('G1') and ' E' in s:
                    return True
                if s.startswith(';TYPE:') or s.startswith(';LAYER_CHANGE'):
                    return True
    except Exception:
        pass
    return False


def run_import(input_path: str, output_csv: str) -> None:
    """
    Convert a slicer G-code file to the universal CSV format.
    Raises RuntimeError on unrecoverable errors.
    """
    factor = get_material_factor()

    state = {
        'X': 0.0, 'Y': 0.0, 'Z': 0.0,
        'A': 0.0, 'B': 0.0, 'C': 0.0,
        'E': 0.0, 'previous_E': 0.0,
        'F_Speed': 0.0,
        'Temperature': 0,
        'FanPwm': 0,
        'LayerIndex': 0,
        'FeatureId': 0,
        'Progress': 0,
    }

    row_count = 0

    try:
        with open(input_path, 'r', encoding='utf-8') as fin, \
             open(output_csv, 'w', encoding='utf-8') as fout:

            fout.write("# Universal Data Stream | Created by: PrusaSlicer G-code Importer\n")
            fout.write("# TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS\n")

            for line in fin:
                line = line.strip()
                if not line:
                    continue

                if line.startswith('G92'):
                    if 'E0' in line:
                        state['E'] = 0.0
                        state['previous_E'] = 0.0

                elif line.startswith('M104') or line.startswith('M109'):
                    m = re.search(r'S(\d+)', line)
                    if m:
                        state['Temperature'] = int(m.group(1))

                elif line.startswith('M106'):
                    m = re.search(r'S(\d+)', line)
                    if m:
                        state['FanPwm'] = int(m.group(1))

                elif line.startswith('M107'):
                    state['FanPwm'] = 0

                elif line.startswith('M73'):
                    m = re.search(r'P(\d+)', line)
                    if m:
                        state['Progress'] = int(m.group(1))

                elif line.startswith(';LAYER_CHANGE'):
                    state['LayerIndex'] += 1

                elif line.startswith(';TYPE:'):
                    state['FeatureId'] = _parse_feature_type(line.split(';TYPE:')[1])

                elif line.startswith('G1'):
                    clean = line.split(';')[0].strip()
                    row = _parse_g1(clean, state, factor)
                    if row:
                        fout.write(row + '\n')
                        row_count += 1

    except FileNotFoundError:
        raise RuntimeError(f"Input file not found: {input_path}")
    except OSError as exc:
        raise RuntimeError(f"I/O error: {exc}")

    if row_count == 0:
        raise RuntimeError("No valid G1 moves found — wrong file format?")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _parse_feature_type(type_string: str) -> int:
    t = type_string.lower().strip()
    if 'external perimeter' in t:
        return 2
    if 'perimeter' in t:
        return 1
    if 'solid infill' in t or 'top solid infill' in t:
        return 3
    if 'infill' in t:
        return 4
    if 'skirt' in t or 'brim' in t:
        return 5
    if 'support' in t:
        return 6
    return 0


def _parse_g1(line: str, state: dict, factor: float):
    old_x, old_y, old_z = state['X'], state['Y'], state['Z']

    for param in line.split()[1:]:
        if param.startswith('X'):
            state['X'] = float(param[1:])
        elif param.startswith('Y'):
            state['Y'] = float(param[1:])
        elif param.startswith('Z'):
            state['Z'] = float(param[1:])
        elif param.startswith('F'):
            state['F_Speed'] = float(param[1:])
        elif param.startswith('E'):
            state['E'] = float(param[1:])

    dist = math.sqrt((state['X'] - old_x) ** 2
                     + (state['Y'] - old_y) ** 2
                     + (state['Z'] - old_z) ** 2)
    delta_e = state['E'] - state['previous_E']
    state['previous_E'] = state['E']

    if dist > 0.001 and delta_e > 0:
        action = 'P'
    elif dist > 0.001:
        action = 'T'
    elif delta_e < 0:
        action = 'R'
    elif delta_e > 0:
        action = 'U'
    else:
        return None  # micro-segment, no movement and no extrusion

    e_ratio = (delta_e / dist * factor) if action == 'P' else 0.0
    tcp_speed = state['F_Speed'] / 60.0
    fan_pct = int(state['FanPwm'] * 100 / 255)

    return (f"{action};{state['X']:.3f};{state['Y']:.3f};{state['Z']:.3f};"
            f"{state['A']:.3f};{state['B']:.3f};{state['C']:.3f};"
            f"{tcp_speed:.3f};{e_ratio:.6f};"
            f"{state['Temperature']};{fan_pct};"
            f"{state['LayerIndex']};{state['FeatureId']};{state['Progress']}")


# ---------------------------------------------------------------------------
# Stand-alone CLI (optional — viewer calls run_import() directly)
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=get_label())
    parser.add_argument('input_file')
    parser.add_argument('-o', '--output_file', default=None)
    args = parser.parse_args()

    out = args.output_file or os.path.splitext(args.input_file)[0] + '.csv'
    print(f"Parsing {args.input_file}  [factor={get_material_factor()}]")
    run_import(args.input_file, out)
    print(f"Done → {out}")
