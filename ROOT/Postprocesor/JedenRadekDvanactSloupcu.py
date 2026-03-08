from ..trajectory_schema import feature_name
from .postprocessor_common import export_rows, run_postprocessor_cli


def get_label() -> str:
    return "Single-line export (12 columns)"


def _format_row(point_no, row):
    rpm = row.tcp_speed * row.e_ratio

    if row.move_type == "P":
        note = f"Print_{feature_name(row.feature, identifier_style=True)}"
    elif row.move_type == "T":
        note = "Travel"
    elif row.move_type == "R":
        note = "Retract"
    elif row.move_type == "U":
        note = "Unretract"
    else:
        note = "Unknown"

    note = note[:32]
    return (
        f"{row.x:.3f} {row.y:.3f} {row.z:.3f} "
        f"{row.a:.3f} {row.b:.3f} {row.c:.3f} "
        f"{row.tcp_speed:.1f} {rpm:.3f} "
        f"{point_no} {row.layer} {row.progress} "
        f"{note}"
    )


def run_export(input_csv, output_txt):
    export_rows(input_csv, output_txt, _format_row)


if __name__ == "__main__":
    run_postprocessor_cli(run_export, get_label())
