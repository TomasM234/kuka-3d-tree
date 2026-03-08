from PyQt6.QtWidgets import QMessageBox

from .error_reporting import get_logger
from .trajectory_schema import apply_planar_edit_transform, rewrite_trajectory_csv


logger = get_logger(__name__)


class ViewerEditMixin:
    def _edit_increment(self, spinbox, delta):
        spinbox.setValue(spinbox.value() + delta)
        self._apply_edit_transform()

    def _apply_edit_transform(self):
        if self._original_points_xyz is None:
            self.lbl_edit_status.setText("No data loaded.")
            return

        dx = self.spin_edit_x.value()
        dy = self.spin_edit_y.value()
        angle_deg = self.spin_edit_z_rot.value()

        points_xyz, orientations_abc = apply_planar_edit_transform(
            self._original_points_xyz,
            self._original_orientations_abc,
            dx,
            dy,
            angle_deg,
        )

        self.points_xyz = points_xyz
        if orientations_abc is not None:
            self.orientations_abc = orientations_abc

        self.lbl_edit_status.setText(f"X {dx:+.1f} mm | Y {dy:+.1f} mm | Z rot {angle_deg:+.1f} deg")

        self.update_plot()
        self.plotter.reset_camera()

    def _reset_edit_transform(self):
        self.spin_edit_x.setValue(0.0)
        self.spin_edit_y.setValue(0.0)
        self.spin_edit_z_rot.setValue(0.0)
        self._apply_edit_transform()
        self.lbl_edit_status.setText("Reset to original.")

    def _save_edit_to_csv(self):
        if self.points_xyz is None or not self.last_file_path:
            self.lbl_edit_status.setText("No data or file loaded.")
            return

        try:
            rewrite_trajectory_csv(self.last_file_path, self.points_xyz, self.orientations_abc)
            self._original_points_xyz = self.points_xyz.copy()
            if self.orientations_abc is not None:
                self._original_orientations_abc = self.orientations_abc.copy()
            self.spin_edit_x.setValue(0.0)
            self.spin_edit_y.setValue(0.0)
            self.spin_edit_z_rot.setValue(0.0)
            self.lbl_edit_status.setText(f"Saved to {self.last_file_path}. Transform reset.")
        except Exception as exc:
            logger.exception("Failed to save edited trajectory to %s", self.last_file_path)
            QMessageBox.critical(self, "Save Error", f"Failed to save CSV:\n{exc}")
            self.lbl_edit_status.setText("Save failed.")
