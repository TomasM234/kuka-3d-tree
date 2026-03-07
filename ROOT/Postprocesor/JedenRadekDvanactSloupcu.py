import sys
import os

from trajectory_schema import feature_name, iter_csv_rows


def get_label() -> str:
    return "Single-line export (12 columns)"


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
                    note = f"Print_{feature_name(row.feature, identifier_style=True)}"
                elif row.move_type == 'T':
                    note = "Travel"
                elif row.move_type == 'R':
                    note = "Retract"
                elif row.move_type == 'U':
                    note = "Unretract"
                else:
                    note = "Unknown"

                note = note[:32]

                out_line = (
                    f"{row.x:.3f} {row.y:.3f} {row.z:.3f} "
                    f"{row.a:.3f} {row.b:.3f} {row.c:.3f} "
                    f"{row.tcp_speed:.1f} {rpm:.3f} "
                    f"{point_no} {row.layer} {row.progress} "
                    f"{note}"
                )
                outfile.write(out_line + "\r\n")

        print(f"Successfully processed {input_csv} to {output_txt}")
    except Exception as e:
        print(f"Error processing file: {e}")
        sys.exit(1)


def process_csv(input_csv, output_txt):
    """Compatibility wrapper for existing tests/scripts."""
    run_export(input_csv, output_txt)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python JedenRadekExport.py <input_csv> <output_txt>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    run_export(input_file, output_file)
