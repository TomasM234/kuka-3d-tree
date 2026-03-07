import os
import sys
import tempfile
import unittest


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from Postprocesor.JedenRadekDvanactSloupcu import process_csv


class PostprocessorTests(unittest.TestCase):
    def test_point_numbers_are_data_row_based(self):
        csv_payload = "\n".join([
            "# comment",
            "# TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS",
            "P;1;2;3;0;0;0;10;2;0;0;0;1;5",
            "T;4;5;6;0;0;0;20;0;0;0;0;0;10",
        ])

        with tempfile.TemporaryDirectory() as tmpdir:
            src_path = os.path.join(tmpdir, "input.csv")
            out_path = os.path.join(tmpdir, "output.txt")
            with open(src_path, "w", encoding="utf-8") as f:
                f.write(csv_payload)

            process_csv(src_path, out_path)

            with open(out_path, "r", encoding="ascii") as f:
                lines = [line.strip() for line in f if line.strip()]

        self.assertEqual(lines[0].split()[8], "0")
        self.assertEqual(lines[1].split()[8], "1")


if __name__ == "__main__":
    unittest.main()
