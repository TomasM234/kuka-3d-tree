import tempfile
import unittest

from ROOT.Importer.parse_gcode import run_import


class ParseGcodeTests(unittest.TestCase):
    def _run_import(self, program_text):
        with tempfile.TemporaryDirectory() as tmpdir:
            src_path = f"{tmpdir}/input.gcode"
            out_path = f"{tmpdir}/output.csv"
            with open(src_path, "w", encoding="utf-8") as f:
                f.write(program_text.strip() + "\n")

            run_import(src_path, out_path)

            with open(out_path, "r", encoding="utf-8") as f:
                return [line.strip() for line in f if line.strip() and not line.startswith("#")]

    def test_first_layer_change_maps_to_layer_zero(self):
        rows = self._run_import(
            """
            ;LAYER_CHANGE
            ;TYPE:Perimeter
            G92 E0
            G1 X0 Y0 Z0 F600
            G1 X10 Y0 Z0 E1.0 F600
            ;LAYER_CHANGE
            G1 X20 Y0 Z0 E2.0 F600
            """
        )

        self.assertEqual(rows[0].split(";")[11], "0")
        self.assertEqual(rows[1].split(";")[11], "1")


if __name__ == "__main__":
    unittest.main()
