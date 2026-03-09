import math
from time import monotonic

import numpy as np
from PyQt6.QtWidgets import QMessageBox

from .error_reporting import get_logger
from .pose_math import kuka_base_to_matrix, matrix_to_kuka_abc
from .viewer_components import TrajectoryTestThread


logger = get_logger(__name__)

IK_CONFIGS = {
    "Default (Zero)": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "Elbow Up / Front": [0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
    "Elbow Down / Front": [0.0, 0.0, -1.57, 0.0, -1.57, 0.0],
    "Elbow Up / Back": [3.14, -1.57, 1.57, 0.0, 1.57, 0.0],
    "Elbow Down / Back": [3.14, 0.0, -1.57, 0.0, -1.57, 0.0],
    "Transport": [0.0, -2.0, 2.0, 0.0, 2.0, 0.0],
}


class ViewerIkControllerMixin:
    @staticmethod
    def _format_duration(seconds):
        if seconds is None:
            return "calculating..."
        total_seconds = max(0, int(round(seconds)))
        hours, remainder = divmod(total_seconds, 3600)
        minutes, secs = divmod(remainder, 60)
        if hours:
            return f"{hours}h {minutes:02d}m {secs:02d}s"
        if minutes:
            return f"{minutes}m {secs:02d}s"
        return f"{secs}s"

    def _reset_traj_test_feedback(self):
        if not hasattr(self, "progress_traj_test"):
            return
        if hasattr(self, "_traj_test_feedback_timer"):
            self._traj_test_feedback_timer.stop()
        self.progress_traj_test.hide()
        self.progress_traj_test.setRange(0, 100)
        self.progress_traj_test.setValue(0)
        self.progress_traj_test.setFormat("%p%")
        self.lbl_traj_test_progress.hide()
        self.lbl_traj_test_progress.setText("")
        self.lbl_traj_test_eta.hide()
        self.lbl_traj_test_eta.setText("")
        self._traj_test_started_at = None
        self._traj_test_last_progress = None

    def _show_traj_test_feedback(self, message):
        self.progress_traj_test.show()
        self.progress_traj_test.setRange(0, 100)
        self.progress_traj_test.setValue(0)
        self.progress_traj_test.setFormat("%p%")
        self.lbl_traj_test_progress.show()
        self.lbl_traj_test_progress.setText(message)
        self.lbl_traj_test_eta.show()
        self.lbl_traj_test_eta.setText("Elapsed: 0s | ETA: calculating...")
        self._traj_test_last_progress = None
        if hasattr(self, "_traj_test_feedback_timer"):
            self._traj_test_feedback_timer.start()

    def _refresh_traj_test_feedback(self, progress=None):
        if progress is not None:
            self._traj_test_last_progress = progress
        progress = self._traj_test_last_progress
        if progress is None:
            return

        total = max(1, int(progress.total))
        completed = max(0, min(int(progress.completed), total))
        elapsed = 0.0 if self._traj_test_started_at is None else monotonic() - self._traj_test_started_at

        if progress.stage == "spawn":
            self.progress_traj_test.setRange(0, 0)
            eta_text = "launching workers..."
        else:
            self.progress_traj_test.setRange(0, 100)
            percent = int(round(completed * 100 / total))
            self.progress_traj_test.setValue(percent)
            self.progress_traj_test.setFormat(f"{percent}%")
            eta_seconds = None
            if completed > 0:
                eta_seconds = elapsed * (total - completed) / completed
            eta_text = self._format_duration(eta_seconds)

        detail = progress.detail or f"Trajectory test progress: {completed} / {total}"
        self.lbl_traj_test_progress.setText(detail)
        self.lbl_traj_test_eta.setText(
            f"Elapsed: {self._format_duration(elapsed)} | ETA: {eta_text}"
        )
        self.lbl_status.setText(detail)

    def _on_traj_test_feedback_timer(self):
        self._refresh_traj_test_feedback()

    def _get_ik_seed(self, target_pos=None):
        if not self.robot_sim.urdf_model:
            return None

        angles = IK_CONFIGS.get(self.ik_config, IK_CONFIGS["Default (Zero)"])
        seed = self.robot_sim.build_seed_from_active_angles(angles)
        if target_pos is not None and seed is not None:
            seed = self.robot_sim.apply_base_azimuth_to_seed(seed, target_pos)
        return seed

    def _get_ik_seed_candidates(self):
        if not self.robot_sim.urdf_model:
            return []

        candidates = []
        seen = set()
        ordered_names = [self.ik_config] + [name for name in IK_CONFIGS if name != self.ik_config]

        for cfg_name in ordered_names:
            seed = self.robot_sim.build_seed_from_active_angles(IK_CONFIGS[cfg_name])
            if seed is None:
                continue
            key = tuple(np.round(seed, 6))
            if key not in seen:
                seen.add(key)
                candidates.append(seed)

        return candidates

    def _reset_ik_tracking(self, force_config_seed=False):
        self.last_ik_solution = None
        self._force_ik_seed_from_config = force_config_seed

    def _update_robot_ik(self, target_pt, target_ori):
        if not self.robot_sim.urdf_model or not self.show_robot:
            return

        try:
            t_point = kuka_base_to_matrix(
                target_pt[0], target_pt[1], target_pt[2], target_ori[0], target_ori[1], target_ori[2]
            )
            t_base = kuka_base_to_matrix(self.base_x, self.base_y, self.base_z, self.base_a, self.base_b, self.base_c)
            t_base_inv = np.linalg.inv(t_base)

            t_tool = kuka_base_to_matrix(self.tool_x, self.tool_y, self.tool_z, self.tool_a, self.tool_b, self.tool_c)
            t_tool_inv = np.linalg.inv(t_tool)

            t_tcp_target = t_base @ t_point
            t_flange_target = t_tcp_target @ t_tool_inv
            target_pos = t_flange_target[0:3, 3] / 1000.0
            target_orientation = t_flange_target[0:3, 0:3]

            if self.last_ik_solution is not None and not self._force_ik_seed_from_config:
                seed_position = self.last_ik_solution
            else:
                seed_position = self._get_ik_seed(target_pos=target_pos)

            ik_solution = self.robot_sim.calculate_ik(
                target_pos,
                target_orientation=target_orientation,
                initial_position=seed_position,
            )
            if ik_solution is None:
                return

            self.last_ik_solution = np.copy(ik_solution)
            self._force_ik_seed_from_config = False

            active_angles = []
            for idx, link in enumerate(self.robot_sim.ik_chain.links):
                if link.name in self.robot_sim.active_joints and len(active_angles) < 6:
                    active_angles.append(math.degrees(ik_solution[idx]))

            if len(active_angles) == 6:
                text_j123 = f"J1-3 deg: A1:{active_angles[0]:.1f} A2:{active_angles[1]:.1f} A3:{active_angles[2]:.1f}"
                text_j456 = f"J4-6 deg: A4:{active_angles[3]:.1f} A5:{active_angles[4]:.1f} A6:{active_angles[5]:.1f}"
                self.lbl_pose_j123.setText(text_j123)
                self.lbl_pose_j456.setText(text_j456)
                self.lbl_pose_j123.setToolTip(text_j123)
                self.lbl_pose_j456.setToolTip(text_j456)

            tcp_x, tcp_y, tcp_z, tcp_a, tcp_b, tcp_c = matrix_to_kuka_abc(t_tcp_target)
            text_xyz = f"TCP mm: X:{tcp_x:.1f} Y:{tcp_y:.1f} Z:{tcp_z:.1f}"
            text_abc = f"TCP deg: A:{tcp_a:.1f} B:{tcp_b:.1f} C:{tcp_c:.1f}"
            self.lbl_pose_xyz.setText(text_xyz)
            self.lbl_pose_abc.setText(text_abc)
            self.lbl_pose_xyz.setToolTip(text_xyz)
            self.lbl_pose_abc.setToolTip(text_abc)

            evaluation = self.robot_sim.evaluate_solution(ik_solution)
            transforms = self.robot_sim.get_forward_transforms(ik_solution)
            for link_name, t_matrix in transforms.items():
                if link_name not in self.robot_actors:
                    continue
                actor = self.robot_actors[link_name]
                actor.user_matrix = t_base_inv @ t_matrix
                if link_name in evaluation.singularities:
                    actor.prop.color = "yellow"
                elif link_name in evaluation.limit_violations:
                    actor.prop.color = "red"
                else:
                    actor.prop.color = "orange"

            t_extruder = kuka_base_to_matrix(
                self.extruder_x, self.extruder_y, self.extruder_z,
                self.extruder_a, self.extruder_b, self.extruder_c
            )
            t_tcp_display = t_base_inv @ t_tcp_target
            t_extruder_display = t_tcp_display @ t_extruder

            self.tcp_axes_actors = self._draw_axes(t_tcp_display, self.tcp_axes_actors, scale=50.0, shaft_radius=0.08, tip_radius=0.15)
            
            if self.show_extruder and self.extruder_stl != "None":
                self.extruder_axes_actors = self._draw_axes(t_extruder_display, self.extruder_axes_actors, scale=30.0, shaft_radius=0.06, tip_radius=0.12)
                if hasattr(self, "extruder_actor") and self.extruder_actor is not None:
                    self.extruder_actor.user_matrix = t_extruder_display
            else:
                if self.extruder_axes_actors:
                    for actor in self.extruder_axes_actors:
                        self.plotter.remove_actor(actor)
                    self.extruder_axes_actors.clear()

        except Exception:
            logger.exception("IK update failed for current trajectory point")

    def run_trajectory_test(self):
        if self.points_xyz is None or not self.robot_sim.urdf_model:
            return

        ticket = self._begin_job("trajectory_test", "Testing trajectory... This might take a while.")
        self._traj_test_started_at = monotonic()
        self._show_traj_test_feedback("Starting trajectory test...")
        base_params = (self.base_x, self.base_y, self.base_z, self.base_a, self.base_b, self.base_c)
        tool_params = (self.tool_x, self.tool_y, self.tool_z, self.tool_a, self.tool_b, self.tool_c)
        seed_candidates = self._get_ik_seed_candidates()
        if not seed_candidates:
            fallback_seed = self._get_ik_seed()
            if fallback_seed is not None:
                seed_candidates = [fallback_seed]

        self.traj_thread = TrajectoryTestThread(
            self.points_xyz,
            self.orientations_abc,
            self.robot_sim,
            base_params,
            tool_params,
            seed_candidates,
        )
        self.traj_thread.progress_signal.connect(
            lambda progress, tk=ticket: self.on_traj_test_progress(tk, progress)
        )
        self.traj_thread.error_signal.connect(lambda message, tk=ticket: self.on_traj_test_error(tk, message))
        self.traj_thread.finished_signal.connect(lambda statuses, tk=ticket: self.on_traj_test_finished(tk, statuses))
        self.traj_thread.start()

    def on_traj_test_progress(self, ticket, progress):
        if not self.task_controller.is_current(ticket):
            return
        self._refresh_traj_test_feedback(progress)

    def on_traj_test_error(self, ticket, message):
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        if hasattr(self, "_traj_test_feedback_timer"):
            self._traj_test_feedback_timer.stop()
        self.progress_traj_test.setRange(0, 100)
        self.progress_traj_test.setFormat("Failed")
        self.lbl_traj_test_progress.setText("Trajectory test failed.")
        self.lbl_traj_test_eta.setText(message)
        self.lbl_status.setText("Trajectory test failed.")
        QMessageBox.critical(self, "Trajectory Test Error", message)
        self._traj_test_started_at = None
        self._restore_interaction_state()

    def on_traj_test_finished(self, ticket, statuses):
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        if hasattr(self, "_traj_test_feedback_timer"):
            self._traj_test_feedback_timer.stop()
        elapsed = 0.0 if self._traj_test_started_at is None else monotonic() - self._traj_test_started_at
        self.progress_traj_test.setRange(0, 100)
        self.progress_traj_test.setValue(100)
        self.progress_traj_test.setFormat("100%")
        self.lbl_traj_test_progress.setText("Trajectory test complete.")
        self.lbl_traj_test_eta.setText(f"Elapsed: {self._format_duration(elapsed)}")
        self.lbl_status.setText(f"Test complete in {self._format_duration(elapsed)}.")
        if statuses is not None and len(statuses) == len(self.points_xyz):
            self.color_strip.set_statuses(statuses)
        self._traj_test_started_at = None
        self._restore_interaction_state()
