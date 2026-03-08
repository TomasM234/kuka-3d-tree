import unittest

import numpy as np

from ROOT.trajectory_render import TrajectoryRenderer
from ROOT.trajectory_schema import COLOR_PRINT, COLOR_TRAVEL


class FakeMesh:
    def __init__(self):
        self.points = None
        self.lines = None
        self.cell_data = {}

    def tube(self, radius, n_sides):
        mesh = FakeMesh()
        mesh.points = self.points
        mesh.lines = self.lines
        mesh.cell_data = {
            key: np.array(value, copy=True) if hasattr(value, "__array__") else value
            for key, value in self.cell_data.items()
        }
        return mesh

    def extract_cells(self, selected):
        mesh = FakeMesh()
        mesh.points = self.points
        mesh.lines = self.lines
        mesh.cell_data = {}
        for key, value in self.cell_data.items():
            arr = np.asarray(value)
            mesh.cell_data[key] = arr[selected] if arr.ndim > 0 else value
        return mesh


class FakeActor:
    pass


class FakePlotter:
    def __init__(self):
        self.added = []
        self.removed = []

    def add_mesh(self, mesh, **kwargs):
        actor = FakeActor()
        actor.mesh = mesh
        actor.kwargs = kwargs
        self.added.append(actor)
        return actor

    def remove_actor(self, actor):
        self.removed.append(actor)


class FakePv:
    @staticmethod
    def PolyData():
        return FakeMesh()

    @staticmethod
    def Sphere(radius, center):
        mesh = FakeMesh()
        mesh.radius = radius
        mesh.center = center
        return mesh


class TrajectoryRenderTests(unittest.TestCase):
    def test_clear_actors_keeps_cached_geometry_but_reset_drops_it(self):
        plotter = FakePlotter()
        renderer = TrajectoryRenderer(plotter, FakePv)
        points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32)
        colors = np.array([COLOR_PRINT, COLOR_TRAVEL], dtype=np.uint8)

        renderer.set_trajectory(points, colors, 1.0)
        self.assertIs(renderer._points_xyz, points)
        self.assertIsNotNone(renderer._print_line_mesh)

        last_point = renderer.render(3, 0.5)
        np.testing.assert_allclose(last_point, points[-1])
        self.assertIsNotNone(renderer.path_actor_print)
        self.assertIsNotNone(renderer.tool_actor)

        renderer.clear_actors()
        self.assertIs(renderer._points_xyz, points)
        self.assertIsNotNone(renderer._print_line_mesh)

        renderer.reset()
        self.assertIsNone(renderer._points_xyz)
        self.assertIsNone(renderer._print_line_mesh)


if __name__ == "__main__":
    unittest.main()
