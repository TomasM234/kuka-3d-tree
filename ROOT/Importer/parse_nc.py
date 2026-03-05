"""
Importer: NC G-code — M3/M5 spindle + S-value extruder control
===============================================================

EXPECTED INPUT FORMAT
---------------------
NC G-code as used by CNC-style 3D printing workflows where the extruder
is controlled like a spindle: M3 starts it, M5 stops it, and the S value
sets the "RPM" (or equivalent speed unit chosen by the CAM software).

Key commands interpreted:
  G0 [X..] [Y..] [Z..] [A..] [B..] [C..] [F..]
      Rapid move (always classified as Travel).
  G1 [X..] [Y..] [Z..] [A..] [B..] [C..] [F..]
      Controlled move. Classified as Print if extruder is ON (M3 active).
  M3  Extruder/spindle ON.
  M5  Extruder/spindle OFF.
  S<value>
      Extruder speed / RPM — standalone line, applied to subsequent moves.
  M104 S<temp> / M109 S<temp>
      Hotend temperature (parsed if present, optional).
  M106 S<0-255> / M107
      Fan control (parsed if present, optional).
  G92
      Axis reset — silently ignored (no E axis in NC convention).
  ; ...
      Comment lines — silently skipped.

Move classification:
  T  Travel  — G0, OR G1 with extruder OFF (M5 active)
  P  Print   — G1 with extruder ON (M3 active) and real XYZ displacement

Note: Retract (R) / Unretract (U) are not used in this format because the
extruder is always fully on (M3) or off (M5) — there is no partial retract.

Layer detection:
  A new layer is recorded whenever a Print move changes Z compared to the
  previous print Z (threshold: 0.01 mm).

OUTPUT CSV FORMAT
-----------------
  # comment lines (ignored by viewer)
  TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS

  TYPE      P / T  (extruder on/off; R and U not used)
  X/Y/Z     TCP position [mm]
  A/B/C     TCP orientation [deg] — taken from G0/G1 if present, else 0.0
  TCP_SPEED feedrate [mm/s]  (F value / 60)
  E_RATIO   S_value × material_factor  (0.0 for Travel)
  TEMP      last known hotend setpoint [°C]
  FAN_PCT   fan speed [0–100]
  LAYER     layer index (0-based, auto-incremented on Z change during print)
  FEATURE   always 0 (N/A) — NC files carry no feature metadata
  PROGRESS  linear estimate [0–100 %], computed in post-processing pass
"""

import argparse
import math
import os
import re
import sys


# ---------------------------------------------------------------------------
# Plugin interface — edit these two values when adapting for a new workflow
# ---------------------------------------------------------------------------

def get_label() -> str:
    """Short human-readable name shown in the viewer's importer combobox."""
    return "NC G-code — M3/M5 spindle + S-value extruder"


def get_material_factor() -> float:
    """
    Multiplier applied to the raw S-value before writing E_RATIO to CSV.

    Raw S-value is typically extruder RPM or an equivalent machine unit.
    The postprocessor (JedenRadekDvanactSloupcu) computes:
        out_RPM = TCP_SPEED [mm/s] × E_RATIO

    Set factor to 1.0 to pass S directly as E_RATIO.
    Adjust if your machine uses a different S convention (e.g. S in rad/s).
    """
    return 1.0


# ---------------------------------------------------------------------------
# Plugin interface — detection and conversion
# ---------------------------------------------------------------------------

def can_handle(file_path: str) -> bool:
    """
    Return True if this file looks like NC G-code with M3/M5 spindle control.
    Scans only the first 200 lines for speed.
    """
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for i, line in enumerate(f):
                if i >= 200:
                    break
                s = line.strip()
                if s.startswith('M3') and (len(s) == 2 or not s[2].isdigit()):
                    return True
                if s == 'M5' or s.startswith('M5 '):
                    return True
                if re.match(r'^S[\d.]+\s*$', s):
                    return True
    except Exception:
        pass
    return False


def run_import(input_path: str, output_csv: str) -> None:
    """
    Convert an NC G-code file to the universal CSV format.
    Raises RuntimeError on unrecoverable errors.
    """
    factor = get_material_factor()

    # --- First pass: extract metadata from header comments ---
    layer_height = ""
    deposition_width = ""
    try:
        with open(input_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line.startswith(';'):
                    if line.startswith(('G', 'M', 'S')):
                        break
                    continue
                m = re.search(r'Layer height:\s*([\d.]+)', line)
                if m:
                    layer_height = m.group(1)
                m = re.search(r'Deposition width:\s*([\d.]+)', line)
                if m:
                    deposition_width = m.group(1)
    except Exception:
        pass

    # --- Main pass: parse G-code ---
    state = {
        'X': 0.0, 'Y': 0.0, 'Z': 0.0,
        'A': 0.0, 'B': 0.0, 'C': 0.0,
        'F_Speed': 0.0,
        'S_value': 0.0,
        'extruder_on': False,
        'Temperature': 0,
        'FanPct': 0,
        'LayerIndex': 0,
        'FeatureId': 0,
        'Progress': 0,
        'last_print_z': -999.0,
    }

    rows = []

    try:
        with open(input_path, 'r', encoding='utf-8') as fin:
            for line in fin:
                line = line.strip()
                if not line or line.startswith(';'):
                    continue

                if line.startswith('M3'):
                    state['extruder_on'] = True
                    continue
                if line.startswith('M5'):
                    state['extruder_on'] = False
                    continue
                if re.match(r'^S[\d.]+', line):
                    m = re.match(r'^S([\d.]+)', line)
                    if m:
                        state['S_value'] = float(m.group(1))
                    continue
                if line.startswith('M104') or line.startswith('M109'):
                    m = re.search(r'S(\d+)', line)
                    if m:
                        state['Temperature'] = int(m.group(1))
                    continue
                if line.startswith('M106'):
                    m = re.search(r'S(\d+)', line)
                    if m:
                        state['FanPct'] = int(int(m.group(1)) * 100 / 255)
                    continue
                if line.startswith('M107'):
                    state['FanPct'] = 0
                    continue
                if line.startswith('G92'):
                    continue
                if line.startswith('G0') or line.startswith('G1'):
                    clean = line.split(';')[0].strip()
                    row = _parse_move(clean, state, factor)
                    if row:
                        rows.append(row)

    except FileNotFoundError:
        raise RuntimeError(f"Input file not found: {input_path}")
    except OSError as exc:
        raise RuntimeError(f"I/O error: {exc}")

    if not rows:
        raise RuntimeError("No valid moves found — wrong file format?")

    # --- Post-process: fill in progress percentages ---
    total = len(rows)
    rows = [f"{r.rsplit(';', 1)[0]};{int(i * 100 / total)}"
            for i, r in enumerate(rows)]

    # --- Write output ---
    src_name = os.path.basename(input_path)
    header_info = f"Source: {src_name}"
    if layer_height:
        header_info += f" | Layer height: {layer_height}"
    if deposition_width:
        header_info += f" | Deposition width: {deposition_width}"

    try:
        with open(output_csv, 'w', encoding='utf-8') as fout:
            fout.write(f"# Universal Data Stream | Created by: NC G-code Importer\n")
            fout.write(f"# {header_info}\n")
            fout.write("# TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS\n")
            for row in rows:
                fout.write(row + '\n')
    except OSError as exc:
        raise RuntimeError(f"Cannot write output file: {exc}")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _parse_move(line: str, state: dict, factor: float):
    """Parse a G0 or G1 line and return a CSV row string, or None."""
    parts = line.split()
    cmd = parts[0]

    old_x, old_y, old_z = state['X'], state['Y'], state['Z']

    for param in parts[1:]:
        key = param[0]
        try:
            val = float(param[1:].replace('=', ''))
        except ValueError:
            continue
        if key == 'X':
            state['X'] = val
        elif key == 'Y':
            state['Y'] = val
        elif key == 'Z':
            state['Z'] = val
        elif key == 'A':
            state['A'] = val
        elif key == 'B':
            state['B'] = val
        elif key == 'C':
            state['C'] = val
        elif key == 'F':
            state['F_Speed'] = val
        # W intentionally ignored

    dist = math.sqrt((state['X'] - old_x) ** 2
                     + (state['Y'] - old_y) ** 2
                     + (state['Z'] - old_z) ** 2)
    if dist < 0.001:
        return None  # no real movement

    if cmd == 'G0':
        action = 'T'
    elif cmd == 'G1' and state['extruder_on']:
        action = 'P'
    else:
        action = 'T'

    # Layer detection: Z change on print moves
    if action == 'P' and abs(state['Z'] - state['last_print_z']) > 0.01:
        state['LayerIndex'] += 1
        state['last_print_z'] = state['Z']

    e_ratio = state['S_value'] * factor if action == 'P' else 0.0
    tcp_speed = state['F_Speed'] / 60.0

    return (f"{action};{state['X']:.3f};{state['Y']:.3f};{state['Z']:.3f};"
            f"{state['A']:.3f};{state['B']:.3f};{state['C']:.3f};"
            f"{tcp_speed:.3f};{e_ratio:.6f};"
            f"{state['Temperature']};{state['FanPct']};"
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
