import argparse
import os

from ..error_reporting import get_logger
from ..trajectory_schema import iter_csv_rows


logger = get_logger(__name__)


def export_rows(input_csv, output_txt, format_row):
    """Write CRLF-terminated lines produced by format_row for each CSV row."""
    if not os.path.exists(input_csv):
        raise FileNotFoundError(f"Input file {input_csv} does not exist.")

    with open(input_csv, "r", encoding="utf-8") as infile, open(output_txt, "w", encoding="ascii", newline="") as outfile:
        for point_no, row in enumerate(iter_csv_rows(infile)):
            lines = format_row(point_no, row)
            if isinstance(lines, str):
                lines = (lines,)
            for line in lines:
                outfile.write(line + "\r\n")


def copy_text_with_crlf(input_file, output_file):
    """Copy a text file while normalizing line endings to CRLF."""
    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file {input_file} does not exist.")

    with open(input_file, "r", encoding="utf-8") as infile, open(output_file, "w", encoding="ascii", newline="") as outfile:
        for line in infile:
            outfile.write(line.rstrip("\n\r") + "\r\n")


def run_postprocessor_cli(run_export, description: str):
    """Shared command-line entry point for postprocessors."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("input_file")
    parser.add_argument("output_file")
    args = parser.parse_args()

    try:
        run_export(args.input_file, args.output_file)
        logger.info("Successfully processed %s to %s", args.input_file, args.output_file)
    except Exception as exc:
        logger.exception("Postprocessor CLI failed for %s", args.input_file)
        raise SystemExit(1) from exc
