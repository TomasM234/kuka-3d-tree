import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from ROOT.trajectory_test_lib import (
    TrajectoryTestConfig,
    TrajectoryTestProgress,
    _build_chunk_seed_handoff,
    _split_chunks,
    run_trajectory_test_parallel,
)


class _FakeRobotSimulator:
    def __init__(self):
        self.ik_chain = SimpleNamespace(links=[object()])

    def load_robot(self, urdf_path, load_meshes=True):
        del load_meshes
        self.urdf_path = urdf_path

    def apply_base_azimuth_to_seed(self, seed_template, target_position_m):
        return np.array(seed_template, dtype=np.float64, copy=True)

    def calculate_ik(self, target_pos, target_orientation=None, initial_position=None):
        del target_orientation, initial_position
        return np.array([target_pos[0]], dtype=np.float64)

    def evaluate_solution(self, solution):
        del solution
        return SimpleNamespace(status=0)


class TrajectoryTestLibTests(unittest.TestCase):
    def test_seed_handoff_uses_previous_chunk_terminal_solution(self):
        points_xyz = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [4.0, 0.0, 0.0]],
            dtype=np.float32,
        )
        orientations_abc = np.zeros((len(points_xyz), 3), dtype=np.float32)
        config = TrajectoryTestConfig(
            urdf_path="fake.urdf",
            base_params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            tool_params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            seed_templates=(np.array([0.0], dtype=np.float64),),
            max_workers=2,
            chunk_size=3,
        )

        chunks = _split_chunks(points_xyz, orientations_abc, max_workers=2, chunk_size=3)

        with patch("ROOT.robot_ik.RobotSimulator", _FakeRobotSimulator):
            seeded_chunks = _build_chunk_seed_handoff(points_xyz, orientations_abc, config, chunks)

        self.assertIsNone(seeded_chunks[0].initial_position)
        self.assertIsNotNone(seeded_chunks[1].initial_position)
        self.assertAlmostEqual(float(seeded_chunks[1].initial_position[0]), 0.002, places=6)

    def test_single_worker_progress_reports_completion(self):
        points_xyz = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
            dtype=np.float32,
        )
        orientations_abc = np.zeros((len(points_xyz), 3), dtype=np.float32)
        config = TrajectoryTestConfig(
            urdf_path="fake.urdf",
            base_params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            tool_params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            seed_templates=(np.array([0.0], dtype=np.float64),),
            max_workers=1,
            chunk_size=2,
        )
        updates = []

        with patch("ROOT.robot_ik.RobotSimulator", _FakeRobotSimulator):
            statuses = run_trajectory_test_parallel(
                points_xyz,
                orientations_abc,
                config,
                progress_callback=updates.append,
            )

        self.assertEqual(len(statuses), len(points_xyz))
        self.assertTrue(updates)
        self.assertIsInstance(updates[0], TrajectoryTestProgress)
        self.assertEqual(updates[0].completed, 0)
        self.assertEqual(updates[-1].stage, "solve")
        self.assertEqual(updates[-1].completed, updates[-1].total)


if __name__ == "__main__":
    unittest.main()
