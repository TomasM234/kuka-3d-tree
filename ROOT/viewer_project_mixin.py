import os

from PyQt6.QtWidgets import QMessageBox, QInputDialog

from .error_reporting import get_logger
from .project_config import DEFAULT_IK_CONFIG, PROJECT_STATE_ATTRS, ProjectConfig


logger = get_logger(__name__)


class ViewerProjectMixin:
    def _apply_project_config_to_state(self, config):
        """Load project-config values into runtime state attributes."""
        for attr_name in PROJECT_STATE_ATTRS:
            setattr(self, attr_name, getattr(config, attr_name))
        self.ik_config = self.ik_config or DEFAULT_IK_CONFIG

    def _build_project_config_from_state(self):
        """Create a persistable project config from runtime state."""
        selected_postprocessor = self.combo_postprocessor.currentData()
        if selected_postprocessor:
            self.last_postprocessor = selected_postprocessor.file_name

        payload = {attr_name: getattr(self, attr_name) for attr_name in PROJECT_STATE_ATTRS}
        payload["ik_config"] = self.ik_config or DEFAULT_IK_CONFIG
        return ProjectConfig(**payload)

    def _load_meta(self):
        """Read viewer_settings.json to determine which project to open."""
        self._project_file = self._project_store.load_last_project_path()
        self._project_name = self._project_store.project_name_from_path(self._project_file)

    def _save_meta(self):
        """Write the current project name to viewer_settings.json."""
        try:
            self._project_store.save_last_project_path(self._project_file)
        except Exception:
            logger.exception("Failed to save viewer metadata")

    def load_settings(self):
        """Load meta + project settings."""
        self._load_meta()
        self._load_project(self._project_file)

    def _load_project(self, path):
        """Load project settings from a JSON file."""
        if not path:
            return
        self._clear_trajectory_state()
        self._clear_robot_state()
        self._project_file = path
        self._project_name = self._project_store.project_name_from_path(path)
        config = self._project_store.load_project(path)
        self._apply_project_config_to_state(config)
        self._apply_project_to_ui()

    def _apply_project_to_ui(self):
        """Push current project values into all UI widgets."""
        self._sync_ui_from_state()
        self._reset_ik_tracking()

        self.draw_table()

        for actor in self.robot_actors.values():
            actor.SetVisibility(self.show_robot)

        if self.points_xyz is not None:
            self._request_update()
        else:
            self.plotter.render()

        self._restore_interaction_state()

    def save_settings(self):
        """Save current project (auto-save)."""
        self._save_project()

    def _save_project(self):
        """Write all workspace settings to the current project file."""
        if not self._project_file:
            return
        try:
            config = self._build_project_config_from_state()
            self._project_store.save_project(self._project_file, config)
        except Exception:
            logger.exception("Failed to save project %s", self._project_file)

    def _new_project(self):
        """Create a new project with default settings."""
        name, ok = QInputDialog.getText(self, "New Project", "Project name:")
        if not ok or not name.strip():
            return
        try:
            path = self._project_store.project_path(name)
        except ValueError:
            QMessageBox.warning(self, "Invalid Name", "Project name cannot be empty.")
            return
        if os.path.exists(path):
            filename = os.path.basename(path)
            QMessageBox.warning(self, "Exists", f"Project '{filename}' already exists.")
            return

        self._save_project()
        self._apply_project_config_to_state(ProjectConfig())
        self._clear_trajectory_state()
        self._clear_robot_state()

        self._project_file = path
        self._project_name = self._project_store.project_name_from_path(path)
        self._save_project()
        self._save_meta()
        self._apply_project_to_ui()
        self.lbl_status.setText(f"New project '{self._project_name}' created.")

    def _open_project(self):
        """Show a list of available projects and load the selected one."""
        projects = self._project_store.list_projects()
        if not projects:
            QMessageBox.information(self, "No Projects", "No projects found in Project/ directory.")
            return

        labels = [self._project_store.project_name_from_path(path) for path in projects]
        current_project_file = os.path.normcase(os.path.abspath(self._project_file))
        current = 0
        for i, project_path in enumerate(projects):
            if os.path.normcase(os.path.abspath(project_path)) == current_project_file:
                current = i
                break

        chosen, ok = QInputDialog.getItem(
            self,
            "Open Project",
            "Available projects:",
            labels,
            current,
            False,
        )
        if not ok:
            return

        self._save_project()

        idx = labels.index(chosen)
        path = projects[idx]
        self._load_project(path)
        self._save_meta()

        if self.last_urdf_path and os.path.exists(self.last_urdf_path):
            self.load_urdf(self.last_urdf_path)

        if self.last_file_path and os.path.exists(self.last_file_path):
            self.load_file(self.last_file_path)

        self.lbl_status.setText(f"Opened project '{self._project_name}'.")

    def _save_project_as(self):
        """Save the current project under a new name."""
        name, ok = QInputDialog.getText(
            self,
            "Save Project As",
            "New project name:",
            text=self._project_name,
        )
        if not ok or not name.strip():
            return
        try:
            path = self._project_store.project_path(name)
        except ValueError:
            QMessageBox.warning(self, "Invalid Name", "Project name cannot be empty.")
            return

        self._project_file = path
        self._project_name = self._project_store.project_name_from_path(path)
        self._save_project()
        self._save_meta()
        self._sync_ui_from_state()
        self.lbl_status.setText(f"Project saved as '{self._project_name}'.")

    def closeEvent(self, event):
        """Save project and meta before closing the application."""
        self._save_project()
        self._save_meta()
        event.accept()
