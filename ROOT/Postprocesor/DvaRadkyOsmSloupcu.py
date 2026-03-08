from ..trajectory_schema import feature_name
from .postprocessor_common import export_rows, run_postprocessor_cli


def get_label() -> str:
    return "Two-line export (8 columns)"


def _format_row(point_no, row):
    rpm = row.tcp_speed * row.e_ratio

    if row.move_type == "P":
        move_type = f"Print ({feature_name(row.feature)})"
    elif row.move_type == "T":
        move_type = "Travel"
    elif row.move_type == "R":
        move_type = "Retract"
    elif row.move_type == "U":
        move_type = "Unretract"
    else:
        move_type = "Unknown"

    line1 = f"Point {point_no}/{row.layer} Progress {row.progress}% Move {move_type}"
    line2 = (
        f"{row.x:.3f} {row.y:.3f} {row.z:.3f} "
        f"{row.a:.3f} {row.b:.3f} {row.c:.3f} "
        f"{row.tcp_speed:.1f} {rpm:.3f}"
    )
    return line1, line2


def run_export(input_csv, output_txt):
    export_rows(input_csv, output_txt, _format_row)


if __name__ == "__main__":
    run_postprocessor_cli(run_export, get_label())
