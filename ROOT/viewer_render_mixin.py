import os


class ViewerRenderMixin:
    def setup_scene(self):
        self.plotter.set_background("#f0f0f0")
        self.plotter.remove_bounding_box()

        self.draw_table()

        origin = (0.0, 0.0, 0.0)
        for direction, color in (((1, 0, 0), "red"), ((0, 1, 0), "green"), ((0, 0, 1), "blue")):
            arrow = self.pv.Arrow(start=origin, direction=direction, scale=50.0, shaft_radius=0.03, tip_radius=0.08)
            self.plotter.add_mesh(arrow, color=color, show_edges=False)

        self.view_isometric()
        self.load_settings()

        if self.last_file_path and os.path.exists(self.last_file_path):
            self.load_file(self.last_file_path)

        if self.last_urdf_path and os.path.exists(self.last_urdf_path):
            self.load_urdf(self.last_urdf_path)

    def view_isometric(self):
        self.plotter.view_isometric()
        self.plotter.camera.zoom(1.2)

    def view_top(self):
        self.plotter.view_xy()
        self.plotter.camera.roll = 0

    def view_side(self):
        self.plotter.view_xz()
        self.plotter.camera.roll = 0

    def on_thickness_changed(self, val):
        if getattr(self, "updating_sliders", False):
            return
        self.print_thickness = float(val) / 10.0
        self.lbl_thickness.setText(f"Extrusion Width:\n{self.print_thickness:.1f} mm")
        self.save_settings()
        if self.points_xyz is not None:
            self.trajectory_renderer.set_trajectory(self.points_xyz, self.colors_rgb, self.print_thickness)
            self._request_update()

    def on_v_slider(self, val):
        if self.updating_sliders or self.points_xyz is None:
            return

        target_step = self.layer_end_indices.get(val, 0)
        self.updating_sliders = True
        self.h_slider.setValue(target_step)
        self.updating_sliders = False

        self.current_layer = val
        self.current_step = target_step
        self.update_ui_labels()
        self._request_update()

    def on_h_slider(self, val):
        if self.updating_sliders or self.points_xyz is None:
            return

        self.current_step = val
        layer_idx = 0
        if val > 0 and self._sorted_layer_ends:
            lo, hi = 0, len(self._sorted_layer_ends) - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if self._sorted_layer_ends[mid][1] < val:
                    lo = mid + 1
                else:
                    hi = mid
            layer_idx = self._sorted_layer_ends[lo][0]

        self.current_layer = layer_idx

        self.updating_sliders = True
        self.v_slider.setValue(layer_idx)
        self.updating_sliders = False

        self.update_ui_labels()
        self._request_update()

    def update_ui_labels(self):
        num_pts = len(self.points_xyz) if self.points_xyz is not None else 0
        self.lbl_h_progress.setText(f"Progress: {self.current_step} / {num_pts} points")
        self.lbl_v_layer.setText(f"Layer:\n{self.current_layer} / {self.max_layer}")

    def _request_update(self):
        self._pending_tube_render = True
        self._render_timer.start()

    def _deferred_update_plot(self):
        if self._pending_tube_render:
            self._pending_tube_render = False
            self.update_plot()

    def update_plot(self):
        self.plotter.suppress_rendering = True
        try:
            self.trajectory_renderer.set_trajectory(self.points_xyz, self.colors_rgb, self.print_thickness)
            if self.points_xyz is None or self.current_step < 2:
                self.trajectory_renderer.clear_actors()
            else:
                last_pt = self.trajectory_renderer.render(self.current_step, self.print_thickness * 0.6)
                if last_pt is not None and self.orientations_abc is not None:
                    last_ori = self.orientations_abc[self.current_step - 1]
                    self._update_robot_ik(last_pt, last_ori)
        finally:
            self.plotter.suppress_rendering = False
            self.plotter.render()
