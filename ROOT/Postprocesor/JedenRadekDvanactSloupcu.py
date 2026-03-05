import sys
import os

def process_csv(input_csv, output_txt):
    if not os.path.exists(input_csv):
        print(f"Error: Input file {input_csv} does not exist.")
        sys.exit(1)

    try:
        with open(input_csv, 'r', encoding='utf-8') as infile, \
             open(output_txt, 'w', encoding='ascii', newline='') as outfile:

            for line_idx, line in enumerate(infile):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                parts = line.split(';')
                if len(parts) >= 14:
                    try:
                        # TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN;LAYER;FEATURE;PROGRESS
                        p_type = parts[0]
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])
                        a = float(parts[4])
                        b = float(parts[5])
                        c = float(parts[6])
                        tcp_speed = float(parts[7])
                        e_ratio = float(parts[8])
                        rpm = tcp_speed * e_ratio
                        layer = int(parts[11])
                        progress = int(parts[13])

                        feature_id = int(parts[12])
                        feature_map = {
                            0: "N/A",
                            1: "Perimeter",
                            2: "External_Perimeter",
                            3: "Solid_Infill",
                            4: "Infill",
                            5: "Skirt_Brim",
                            6: "Support"
                        }

                        if p_type == 'P':
                            feature_name = feature_map.get(feature_id, "Unknown")
                            note = f"Print_{feature_name}"
                        elif p_type == 'T':
                            note = "Travel"
                        elif p_type == 'R':
                            note = "Retract"
                        elif p_type == 'U':
                            note = "Unretract"
                        else:
                            note = "Unknown"

                        # Truncate note to 32 chars (CHAR NOTE[32])
                        note = note[:32]

                        out_line = (
                            f"{x:.3f} {y:.3f} {z:.3f} "
                            f"{a:.3f} {b:.3f} {c:.3f} "
                            f"{tcp_speed:.1f} {rpm:.3f} "
                            f"{line_idx} {layer} {progress} "
                            f"{note}"
                        )
                        outfile.write(out_line + "\r\n")
                    except ValueError as ve:
                        print(f"Skipping line {line_idx} due to value error: {ve}")

        print(f"Successfully processed {input_csv} to {output_txt}")
    except Exception as e:
        print(f"Error processing file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python JedenRadekExport.py <input_csv> <output_txt>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    process_csv(input_file, output_file)
