import argparse
import os

from ..error_reporting import get_logger


logger = get_logger(__name__)


UNIVERSAL_CSV_HEADER = "TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS"


def detect_with_matchers(file_path: str, matchers, max_lines: int = 200) -> bool:
    """Return True when any matcher accepts one of the first max_lines lines."""
    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as file_obj:
            for index, line in enumerate(file_obj):
                if index >= max_lines:
                    break
                stripped = line.strip()
                if any(matcher(stripped) for matcher in matchers):
                    return True
    except Exception:
        logger.exception("Importer detection failed for %s", file_path)
    return False


def write_universal_csv_header(file_obj, created_by: str, header_info: str | None = None):
    """Write the standard universal CSV header block."""
    file_obj.write(f"# Universal Data Stream | Created by: {created_by}\n")
    if header_info:
        file_obj.write(f"# {header_info}\n")
    file_obj.write(f"# {UNIVERSAL_CSV_HEADER}\n")


def run_importer_cli(label: str, run_import, material_factor_fn=None):
    """Shared command-line entry point for import plugins."""
    parser = argparse.ArgumentParser(description=label)
    parser.add_argument("input_file")
    parser.add_argument("-o", "--output_file", default=None)
    args = parser.parse_args()

    output_file = args.output_file or os.path.splitext(args.input_file)[0] + ".csv"
    try:
        if material_factor_fn is None:
            logger.info("Parsing %s", args.input_file)
        else:
            logger.info("Parsing %s  [factor=%s]", args.input_file, material_factor_fn())
        run_import(args.input_file, output_file)
        logger.info("Done -> %s", output_file)
    except Exception as exc:
        logger.exception("Importer CLI failed for %s", args.input_file)
        raise SystemExit(1) from exc
