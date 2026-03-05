import argparse
import sys
import re
import math
import os


def parse_args():
    parser = argparse.ArgumentParser(description="Convert NC G-Code (M3/M5/S extruder) to Universal Robot CSV format.")
    parser.add_argument("input_file", help="Path to the input .nc file")
    parser.add_argument("-o", "--output_file", help="Path to the output .csv file (optional)")
    return parser.parse_args()


def parse_move(line, state):
    """Parse a G0 or G1 movement line and return a CSV row (or None)."""
    parts = line.split()
    cmd = parts[0]  # G0 or G1

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
        # W is intentionally ignored

    dist = math.sqrt((state['X'] - old_x)**2 + (state['Y'] - old_y)**2 + (state['Z'] - old_z)**2)

    if dist < 0.001:
        # No real movement — skip
        return None

    # Determine action type
    if cmd == 'G0':
        action = 'T'  # Travel (rapid move, extruder always off)
    elif cmd == 'G1' and state['extruder_on']:
        action = 'P'  # Print
    else:
        action = 'T'  # Travel (G1 without extruder)

    # Layer detection: Z change on print moves
    if action == 'P' and abs(state['Z'] - state['last_print_z']) > 0.01:
        state['LayerIndex'] += 1
        state['last_print_z'] = state['Z']

    # Calculate values
    e_ratio = state['S_value'] if action == 'P' else 0.0
    tcp_speed_mms = state['F_Speed'] / 60.0

    # Progress estimate (will be patched in post-processing pass)
    progress = state['Progress']

    # TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS
    row = (f"{action};{state['X']:.3f};{state['Y']:.3f};{state['Z']:.3f};"
           f"{state['A']:.3f};{state['B']:.3f};{state['C']:.3f};"
           f"{tcp_speed_mms:.3f};{e_ratio:.3f};"
           f"{state['Temperature']};{state['FanPct']};"
           f"{state['LayerIndex']};{state['FeatureId']};{progress}")
    return row


def main():
    args = parse_args()

    input_file = args.input_file
    output_file = args.output_file if args.output_file else os.path.splitext(input_file)[0] + '.csv'

    print(f"Parsing NC file: {input_file}...")

    # --- First pass: extract metadata from header comments ---
    layer_height = ""
    deposition_width = ""
    try:
        with open(input_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line.startswith(';'):
                    if line.startswith('G') or line.startswith('M') or line.startswith('S'):
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
        'last_print_z': -999.0,  # sentinel for first-layer detection
    }

    rows = []

    try:
        with open(input_file, 'r', encoding='utf-8') as fin:
            for line in fin:
                line = line.strip()
                if not line or line.startswith(';'):
                    continue

                # Extruder spindle ON
                if line.startswith('M3'):
                    state['extruder_on'] = True
                    continue

                # Extruder spindle OFF
                if line.startswith('M5'):
                    state['extruder_on'] = False
                    continue

                # Spindle speed (RPM) — typically on its own line
                if re.match(r'^S[\d.]+', line):
                    m = re.match(r'^S([\d.]+)', line)
                    if m:
                        state['S_value'] = float(m.group(1))
                    continue

                # Temperature commands (if present)
                if line.startswith('M104') or line.startswith('M109'):
                    m = re.search(r'S(\d+)', line)
                    if m:
                        state['Temperature'] = int(m.group(1))
                    continue

                # Fan commands (if present)
                if line.startswith('M106'):
                    m = re.search(r'S(\d+)', line)
                    if m:
                        state['FanPct'] = int((int(m.group(1)) * 100) / 255)
                    continue
                if line.startswith('M107'):
                    state['FanPct'] = 0
                    continue

                # Extruder reset (if present)
                if line.startswith('G92'):
                    continue

                # Movement commands
                if line.startswith('G0') or line.startswith('G1'):
                    # Strip inline comments
                    clean_line = line.split(';')[0].strip()
                    row = parse_move(clean_line, state)
                    if row:
                        rows.append(row)

    except FileNotFoundError:
        print(f"Error: Input file {input_file} not found.")
        sys.exit(1)

    # --- Post-process: compute progress percentages ---
    total_rows = len(rows)
    if total_rows > 0:
        for i in range(total_rows):
            pct = int((i * 100) / total_rows)
            # Replace the last field (PROGRESS) in each row
            parts = rows[i].rsplit(';', 1)
            rows[i] = f"{parts[0]};{pct}"

    # --- Write output ---
    try:
        with open(output_file, 'w', encoding='utf-8') as fout:
            # Header
            src_name = os.path.basename(input_file)
            header_info = f"Zdrojovy soubor: {src_name}"
            if layer_height:
                header_info += f" | Layer height: {layer_height}"
            if deposition_width:
                header_info += f" | Deposition width: {deposition_width}"

            fout.write(f"# Universal Data Stream pro Robota | Vytvoreno: NC Parser v1.0\n")
            fout.write(f"# {header_info}\n")
            fout.write("# TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS\n")

            for row in rows:
                fout.write(row + '\n')

    except IOError as e:
        print(f"Error writing output file: {e}")
        sys.exit(1)

    print(f"Done! {total_rows} data points written to {output_file}.")


if __name__ == '__main__':
    main()
