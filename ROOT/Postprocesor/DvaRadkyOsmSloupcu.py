import sys
import os

from trajectory_schema import feature_name, iter_csv_rows


def get_label() -> str:
    return "Two-line export (8 columns)"


def run_export(input_csv, output_txt):
    if not os.path.exists(input_csv):
        print(f"Error: Input file {input_csv} does not exist.")
        sys.exit(1)
        
    try:
        with open(input_csv, 'r', encoding='utf-8') as infile, \
             open(output_txt, 'w', encoding='ascii', newline='') as outfile:
            for point_no, row in enumerate(iter_csv_rows(infile)):
                rpm = row.tcp_speed * row.e_ratio

                if row.move_type == 'P':
                    move_type = f"Print ({feature_name(row.feature)})"
                elif row.move_type == 'T':
                    move_type = "Travel"
                elif row.move_type == 'R':
                    move_type = "Retract"
                elif row.move_type == 'U':
                    move_type = "Unretract"
                else:
                    move_type = "Unknown"

                line1 = f"Point {point_no}/{row.layer} Progress {row.progress}% Move {move_type}"
                line2 = (
                    f"{row.x:.3f} {row.y:.3f} {row.z:.3f} "
                    f"{row.a:.3f} {row.b:.3f} {row.c:.3f} "
                    f"{row.tcp_speed:.1f} {rpm:.3f}"
                )

                outfile.write(line1 + "\r\n")
                outfile.write(line2 + "\r\n")
                        
        print(f"Successfully processed {input_csv} to {output_txt}")
    except Exception as e:
        print(f"Error processing file: {e}")
        sys.exit(1)


def process_csv_advanced(input_csv, output_txt):
    """Compatibility wrapper for existing tests/scripts."""
    run_export(input_csv, output_txt)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python complex_export.py <input_csv> <output_txt>")
        sys.exit(1)
        
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    run_export(input_file, output_file)
