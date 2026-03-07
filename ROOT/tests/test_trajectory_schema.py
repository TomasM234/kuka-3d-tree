import os
import sys
import tempfile
import unittest


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from trajectory_schema import parse_csv_row, rewrite_trajectory_csv


class TrajectorySchemaTests(unittest.TestCase):
    def test_parse_csv_row_supports_legacy_format(self):
        row = parse_csv_row("T;1;2;3;120;0.5;0;0;7")

        self.assertEqual(row.move_type, "T")
        self.assertEqual(row.a, 0.0)
        self.assertEqual(row.tcp_speed, 120.0)
        self.assertEqual(row.layer, 7)

    def test_rewrite_trajectory_csv_validates_point_count(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            file_path = os.path.join(tmpdir, "sample.csv")
            with open(file_path, "w", encoding="utf-8") as f:
                f.write("# header\n")
                f.write("P;0;0;0;0;0;0;10;1;0;0;0;0;0\n")

            with self.assertRaises(ValueError):
                rewrite_trajectory_csv(
                    file_path,
                    [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
                    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                )


if __name__ == "__main__":
    unittest.main()
