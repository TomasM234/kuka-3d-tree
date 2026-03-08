from ..Postprocesor.postprocessor_common import copy_text_with_crlf, run_postprocessor_cli


def get_label() -> str:
    return "Raw CSV copy with CRLF line endings"


def run_export(input_csv, output_txt):
    copy_text_with_crlf(input_csv, output_txt)


if __name__ == "__main__":
    run_postprocessor_cli(run_export, get_label())
