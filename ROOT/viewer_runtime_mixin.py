class ViewerRuntimeMixin:
    def _set_status_text(self, text):
        self.lbl_status.setText(text)

    def _on_busy_changed(self, busy):
        self.left_panel.setEnabled(not busy)
        self.h_slider.setEnabled(not busy and self.points_xyz is not None and len(self.points_xyz) > 0)
        self.v_slider.setEnabled(not busy and self.points_xyz is not None and len(self.points_xyz) > 0)
        self.btn_test_traj.setEnabled(
            not busy
            and self.points_xyz is not None
            and len(self.points_xyz) > 0
            and self.robot_sim.urdf_model is not None
        )

    def _begin_job(self, name, status_text):
        """Lock interactive controls while an async job is in flight."""
        return self.task_controller.begin(name, status_text)

    def _end_job(self, ticket):
        """Release the async job lock and restore control enablement."""
        self.task_controller.finish(ticket)

    def _restore_interaction_state(self):
        """Restore control enablement based on loaded data and current busy state."""
        self._on_busy_changed(self.task_controller.is_busy())

    def _reset_pose_labels(self):
        """Reset the live pose labels when no robot/trajectory is active."""
        defaults = [
            (self.lbl_pose_xyz, "TCP mm: X:- Y:- Z:-"),
            (self.lbl_pose_abc, "TCP deg: A:- B:- C:-"),
            (self.lbl_pose_j123, "J1-3 deg: A1:- A2:- A3:-"),
            (self.lbl_pose_j456, "J4-6 deg: A4:- A5:- A6:-"),
        ]
        for label, text in defaults:
            label.setText(text)
            label.setToolTip(text)

    def _clear_path_actors(self):
        """Remove rendered trajectory actors from the scene."""
        self.trajectory_renderer.reset()

    def _clear_trajectory_state(self):
        """Drop loaded trajectory data and reset related UI state."""
        self.points_xyz = None
        self._original_points_xyz = None
        self._original_orientations_abc = None
        self.orientations_abc = None
        self.colors_rgb = None
        self.layer_end_indices = {}
        self._sorted_layer_ends = []
        self.max_layer = 0
        self.current_step = 0
        self.current_layer = 0
        self.estimated_time_s = 0.0
        self.estimated_weight_g = 0.0
        self._clear_path_actors()
        self.color_strip.set_statuses(None)
        self.lbl_print_time.setText("")
        self.lbl_print_weight.setText("")
        self.lbl_edit_status.setText("")
        self.spin_edit_x.setValue(0.0)
        self.spin_edit_y.setValue(0.0)
        self.spin_edit_z_rot.setValue(0.0)
        if hasattr(self, "_reset_traj_test_feedback"):
            self._reset_traj_test_feedback()
        self.h_slider.setValue(0)
        self.h_slider.setEnabled(False)
        self.v_slider.setValue(0)
        self.v_slider.setEnabled(False)
        self.update_ui_labels()
        self._reset_ik_tracking()

    def _clear_robot_state(self):
        """Drop loaded robot data and reset related UI state."""
        for actor in self.robot_actors.values():
            self.plotter.remove_actor(actor)
        self.robot_actors.clear()
        self.robot_sim = self.robot_simulator_cls()
        self.lbl_robot_name.setText("No robot selected.")
        self._reset_pose_labels()
        self._reset_ik_tracking()
