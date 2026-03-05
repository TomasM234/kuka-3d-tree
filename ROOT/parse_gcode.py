import argparse
import sys
import re
import math
import os

def parse_args():
    parser = argparse.ArgumentParser(description="Convert G-Code to Universal Robot CSV format.")
    parser.add_argument("input_file", help="Path to the input .gcode file")
    parser.add_argument("-o", "--output_file", help="Path to the output .csv file (optional)")
    return parser.parse_args()

def parse_feature_type(type_string):
    type_string = type_string.lower().strip()
    if 'external perimeter' in type_string:
        return 2
    elif 'perimeter' in type_string:
        return 1
    elif 'solid infill' in type_string or 'top solid infill' in type_string:
        return 3
    elif 'infill' in type_string:
        return 4
    elif 'skirt' in type_string or 'brim' in type_string:
        return 5
    elif 'support' in type_string:
        return 6
    return 0

def parse_g1(line, state):
    # Extracts parameters from G1 line
    params = line.split()
    
    # Temporarily store old positions
    old_x, old_y, old_z = state['X'], state['Y'], state['Z']
    
    for param in params[1:]:
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
            
    # Calculate Euclidean distance
    dist = math.sqrt((state['X'] - old_x)**2 + (state['Y'] - old_y)**2 + (state['Z'] - old_z)**2)
    
    # Calculate delta E
    delta_e = state['E'] - state['previous_E']
    state['previous_E'] = state['E']
    
    # Determine Action Type
    action = '?'
    if dist > 0.001 and delta_e > 0:
        action = 'P' # Print
    elif dist > 0.001 and delta_e <= 0:
        action = 'T' # Travel
    elif dist <= 0.001 and delta_e < 0:
        action = 'R' # Retract
    elif dist <= 0.001 and delta_e > 0:
        action = 'U' # Unretract
    elif dist <= 0.001 and delta_e == 0:
        # Micro-segment or pure Z hop without movement, safely ignore or mark as T with dist=0
        return None
        
    # Calculate constants
    e_ratio = delta_e / dist if action == 'P' else 0.0
    tcp_speed_mms = state['F_Speed'] / 60.0
    fan_pct = int((state['FanPwm'] * 100) / 255)
    
    # Format the CSV row
    # TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS
    row = f"{action};{state['X']:.3f};{state['Y']:.3f};{state['Z']:.3f};{state['A']:.3f};{state['B']:.3f};{state['C']:.3f};{tcp_speed_mms:.3f};{e_ratio:.3f};{state['Temperature']};{fan_pct};{state['LayerIndex']};{state['FeatureId']};{state['Progress']}"
    return row

def main():
    args = parse_args()
    
    input_file = args.input_file
    output_file = args.output_file if args.output_file else os.path.splitext(input_file)[0] + '.csv'
    
    print(f"Parsing {input_file}...")
    
    # Initialize State Machine
    state = {
        'X': 0.0, 'Y': 0.0, 'Z': 0.0,
        'A': 0.0, 'B': 0.0, 'C': 0.0,
        'E': 0.0, 'previous_E': 0.0,
        'F_Speed': 0.0,
        'Temperature': 0,
        'FanPwm': 0,
        'LayerIndex': 0,
        'FeatureId': 0,
        'Progress': 0
    }
    
    row_count = 0
    
    try:
        with open(input_file, 'r', encoding='utf-8') as fin, \
             open(output_file, 'w', encoding='utf-8') as fout:
            # Write header
            fout.write("# Universal Data Stream pro Robota | Vytvoreno: G-Code Parser v1.0\n")
            fout.write("# TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS\n")
            
            for line in fin:
                line = line.strip()
                if not line:
                    continue
                
                # Handling non-move commands
                if line.startswith('G92'):
                    if 'E0' in line:
                        state['E'] = 0.0
                        state['previous_E'] = 0.0
                elif line.startswith('M104') or line.startswith('M109'):
                    match = re.search(r'S(\d+)', line)
                    if match:
                        state['Temperature'] = int(match.group(1))
                elif line.startswith('M106'):
                    match = re.search(r'S(\d+)', line)
                    if match:
                        state['FanPwm'] = int(match.group(1))
                elif line.startswith('M107'):
                    state['FanPwm'] = 0
                elif line.startswith('M73'):
                    match = re.search(r'P(\d+)', line)
                    if match:
                        state['Progress'] = int(match.group(1))
                elif line.startswith(';LAYER_CHANGE'):
                    state['LayerIndex'] += 1
                elif line.startswith(';TYPE:'):
                    type_str = line.split(';TYPE:')[1]
                    state['FeatureId'] = parse_feature_type(type_str)
                    
                # Handling movement
                elif line.startswith('G1'):
                    # Strip comments if any exist on the line
                    clean_line = line.split(';')[0].strip()
                    row = parse_g1(clean_line, state)
                    if row:
                        fout.write(row + '\n')
                        row_count += 1
                        
    except FileNotFoundError:
        print(f"Error: Input file {input_file} not found.")
        sys.exit(1)
        
    print(f"Done! {row_count} data points written to {output_file}.")

if __name__ == '__main__':
    main()
