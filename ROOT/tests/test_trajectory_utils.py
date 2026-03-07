import os
import sys
import unittest

import numpy as np


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from trajectory_utils import apply_planar_edit_transform, parse_csv_row


class TrajectoryUtilsTests(unittest.TestCase):
    def test_planar_transform_rotates_points_and_orientations(self):
        points = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32)
        orientations = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)

        new_points, new_orientations = apply_planar_edit_transform(points, orientations, 10.0, 5.0, 90.0)

        np.testing.assert_allclose(new_points[0], [11.0, 4.0, 0.0], atol=1e-5)
        np.testing.assert_allclose(new_points[1], [11.0, 6.0, 0.0], atol=1e-5)
        self.assertAlmostEqual(float(new_orientations[0][0]), 90.0, places=4)
        self.assertAlmostEqual(float(new_orientations[1][0]), 90.0, places=4)

    def test_parse_csv_row_skips_comments(self):
        self.assertIsNone(parse_csv_row("# comment"))
        row = parse_csv_row("P;1;2;3;0;0;0;10;2;200;50;1;4;99")
        self.assertEqual(row.move_type, "P")
        self.assertEqual(row.feature, 4)


if __name__ == "__main__":
    unittest.main()
