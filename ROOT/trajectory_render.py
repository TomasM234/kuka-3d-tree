import numpy as np

from trajectory_schema import COLOR_PRINT


class TrajectoryRenderer:
    """Caches full trajectory meshes and renders step-limited subsets."""

    def __init__(self, plotter, pv_module):
        self.plotter = plotter
        self.pv = pv_module
        self.path_actor_print = None
        self.path_actor_travel = None
        self.tool_actor = None
        self._points_xyz = None
        self._colors_rgb = None
        self._thickness = None
        self._print_segment_indices = np.zeros(0, dtype=np.int32)
        self._travel_segment_indices = np.zeros(0, dtype=np.int32)
        self._print_line_mesh = None
        self._travel_line_mesh = None
        self._print_tube_mesh = None
        self._tube_supports_segment_filter = False

    def clear_actors(self):
        for attr in ("path_actor_print", "path_actor_travel", "tool_actor"):
            actor = getattr(self, attr, None)
            if actor:
                self.plotter.remove_actor(actor)
                setattr(self, attr, None)

    def reset(self):
        self.clear_actors()
        self._print_line_mesh = None
        self._travel_line_mesh = None
        self._print_tube_mesh = None
        self._tube_supports_segment_filter = False
        self._points_xyz = None
        self._colors_rgb = None
        self._thickness = None
        self._print_segment_indices = np.zeros(0, dtype=np.int32)
        self._travel_segment_indices = np.zeros(0, dtype=np.int32)

    def clear(self):
        """Backward-compatible full reset used when trajectory data is dropped."""
        self.reset()

    def set_trajectory(self, points_xyz, colors_rgb, thickness):
        if points_xyz is None or colors_rgb is None:
            self.reset()
            return

        points_changed = (
            self._points_xyz is not points_xyz or
            self._colors_rgb is not colors_rgb
        )
        thickness_changed = self._thickness != thickness

        self._points_xyz = points_xyz
        self._colors_rgb = colors_rgb
        self._thickness = thickness

        if points_changed:
            self._build_line_meshes()
            self._print_tube_mesh = None
            self._tube_supports_segment_filter = False
        elif thickness_changed:
            self._print_tube_mesh = None
            self._tube_supports_segment_filter = False

    def render(self, current_step, tool_radius):
        if self._points_xyz is None or current_step < 2:
            self.clear_actors()
            return None

        self._ensure_tube_cache()
        max_segment = current_step - 1
        sub_points = self._points_xyz[:current_step]

        print_idx = self._print_segment_indices[self._print_segment_indices < max_segment]
        travel_idx = self._travel_segment_indices[self._travel_segment_indices < max_segment]

        if self.path_actor_print:
            self.plotter.remove_actor(self.path_actor_print)
            self.path_actor_print = None
        print_mesh = self._extract_print_subset(print_idx, sub_points)
        if print_mesh is not None:
            self.path_actor_print = self.plotter.add_mesh(
                print_mesh,
                scalars="Colors",
                rgb=True,
                reset_camera=False,
            )

        if self.path_actor_travel:
            self.plotter.remove_actor(self.path_actor_travel)
            self.path_actor_travel = None
        travel_mesh = self._extract_travel_subset(travel_idx, sub_points)
        if travel_mesh is not None:
            self.path_actor_travel = self.plotter.add_mesh(
                travel_mesh,
                scalars="Colors",
                rgb=True,
                line_width=1.0,
                reset_camera=False,
            )

        if self.tool_actor:
            self.plotter.remove_actor(self.tool_actor)
        last_pt = self._points_xyz[current_step - 1]
        tool_mesh = self.pv.Sphere(radius=tool_radius, center=last_pt)
        self.tool_actor = self.plotter.add_mesh(tool_mesh, color="cyan", reset_camera=False)
        return last_pt

    def _build_line_meshes(self):
        self.clear_actors()
        self._print_line_mesh = None
        self._travel_line_mesh = None
        self._print_tube_mesh = None
        self._tube_supports_segment_filter = False
        self._print_segment_indices = np.zeros(0, dtype=np.int32)
        self._travel_segment_indices = np.zeros(0, dtype=np.int32)
        if self._points_xyz is None or self._colors_rgb is None or len(self._colors_rgb) == 0:
            return

        is_print_move = np.all(self._colors_rgb == COLOR_PRINT, axis=1)
        self._print_segment_indices = np.flatnonzero(is_print_move).astype(np.int32)
        self._travel_segment_indices = np.flatnonzero(~is_print_move).astype(np.int32)

        if len(self._print_segment_indices) > 0:
            lines = np.empty((len(self._print_segment_indices), 3), dtype=np.int32)
            lines[:, 0] = 2
            lines[:, 1] = self._print_segment_indices
            lines[:, 2] = self._print_segment_indices + 1
            mesh = self.pv.PolyData()
            mesh.points = self._points_xyz
            mesh.lines = lines.ravel()
            mesh.cell_data["Colors"] = self._colors_rgb[self._print_segment_indices]
            mesh.cell_data["segment_idx"] = self._print_segment_indices
            self._print_line_mesh = mesh

        if len(self._travel_segment_indices) > 0:
            lines = np.empty((len(self._travel_segment_indices), 3), dtype=np.int32)
            lines[:, 0] = 2
            lines[:, 1] = self._travel_segment_indices
            lines[:, 2] = self._travel_segment_indices + 1
            mesh = self.pv.PolyData()
            mesh.points = self._points_xyz
            mesh.lines = lines.ravel()
            mesh.cell_data["Colors"] = self._colors_rgb[self._travel_segment_indices]
            mesh.cell_data["segment_idx"] = self._travel_segment_indices
            self._travel_line_mesh = mesh

    def _ensure_tube_cache(self):
        if self._print_line_mesh is None or self._print_tube_mesh is not None:
            return
        try:
            self._print_tube_mesh = self._print_line_mesh.tube(radius=self._thickness / 2.0, n_sides=8)
            self._tube_supports_segment_filter = "segment_idx" in self._print_tube_mesh.cell_data
        except Exception:
            self._print_tube_mesh = None
            self._tube_supports_segment_filter = False

    def _extract_print_subset(self, print_idx, sub_points):
        if len(print_idx) == 0:
            return None

        if self._tube_supports_segment_filter and self._print_tube_mesh is not None:
            tube_segment_idx = np.asarray(self._print_tube_mesh.cell_data["segment_idx"])
            selected = np.flatnonzero(tube_segment_idx < len(sub_points) - 1)
            if len(selected) > 0:
                return self._print_tube_mesh.extract_cells(selected)

        lines = np.empty((len(print_idx), 3), dtype=np.int32)
        lines[:, 0] = 2
        lines[:, 1] = print_idx
        lines[:, 2] = print_idx + 1
        mesh = self.pv.PolyData()
        mesh.points = sub_points
        mesh.lines = lines.ravel()
        mesh.cell_data["Colors"] = self._colors_rgb[print_idx]
        try:
            return mesh.tube(radius=self._thickness / 2.0, n_sides=8)
        except Exception:
            return mesh

    def _extract_travel_subset(self, travel_idx, sub_points):
        if len(travel_idx) == 0:
            return None
        lines = np.empty((len(travel_idx), 3), dtype=np.int32)
        lines[:, 0] = 2
        lines[:, 1] = travel_idx
        lines[:, 2] = travel_idx + 1
        mesh = self.pv.PolyData()
        mesh.points = sub_points
        mesh.lines = lines.ravel()
        mesh.cell_data["Colors"] = self._colors_rgb[travel_idx]
        return mesh
