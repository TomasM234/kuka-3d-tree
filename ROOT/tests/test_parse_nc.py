import tempfile
import unittest

from ROOT.Importer.parse_nc import run_import


class ParseNcTests(unittest.TestCase):
    def _run_import(self, program_text):
        with tempfile.TemporaryDirectory() as tmpdir:
            src_path = f"{tmpdir}/input.nc"
            out_path = f"{tmpdir}/output.csv"
            with open(src_path, "w", encoding="utf-8") as f:
                f.write(program_text.strip() + "\n")

            run_import(src_path, out_path)

            with open(out_path, "r", encoding="utf-8") as f:
                return [line.strip() for line in f if line.strip() and not line.startswith("#")]

    def test_m50_does_not_disable_extruder_and_progress_reaches_100(self):
        rows = self._run_import(
            """
            M3
            S60
            G1 X10 Y0 Z0 F600
            M50
            G1 X20 Y0 Z0 F600
            M5
            G1 X30 Y0 Z0 F600
            """
        )

        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0].split(";")[0], "P")
        self.assertEqual(rows[1].split(";")[0], "P")
        self.assertEqual(rows[2].split(";")[0], "T")
        self.assertEqual(rows[0].split(";")[11], "0")
        self.assertEqual(rows[-1].split(";")[13], "100")

    def test_orientation_only_move_is_preserved(self):
        rows = self._run_import(
            """
            M3
            S60
            G1 X10 Y0 Z0 F600
            G1 X10 Y0 Z0 A90 F600
            """
        )

        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[1].split(";")[0], "T")
        self.assertEqual(rows[1].split(";")[4], "90.000")


if __name__ == "__main__":
    unittest.main()
