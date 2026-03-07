import os

from PyQt6.QtWidgets import QMessageBox, QInputDialog

from project_config import DEFAULT_IK_CONFIG, ProjectConfig


class ViewerProjectMixin:
    def _apply_project_config_to_state(self, config):
        """Load project-config values into runtime state attributes."""
        self.print_thickness = config.print_thickness
        self.last_file_path = config.last_file_path
        self.last_postprocessor = config.last_postprocessor
        self.last_urdf_path = config.last_urdf_path

        self.base_x = config.base_x
        self.base_y = config.base_y
        self.base_z = config.base_z
        self.base_a = config.base_a
        self.base_b = config.base_b
        self.base_c = config.base_c

        self.tool_x = config.tool_x
        self.tool_y = config.tool_y
        self.tool_z = config.tool_z
        self.tool_a = config.tool_a
        self.tool_b = config.tool_b
        self.tool_c = config.tool_c

        self.table_x1 = config.table_x1
        self.table_y1 = config.table_y1
        self.table_x2 = config.table_x2
        self.table_y2 = config.table_y2
        self.show_table = config.show_table
        self.show_robot = config.show_robot
        self.ik_config = config.ik_config or DEFAULT_IK_CONFIG

    def _build_project_config_from_state(self):
        """Create a persistable project config from runtime state."""
        selected_postprocessor = self.combo_postprocessor.currentData()
        if selected_postprocessor:
            self.last_postprocessor = selected_postprocessor.file_name

        return ProjectConfig(
            print_thickness=self.print_thickness,
            last_file_path=self.last_file_path,
            last_postprocessor=self.last_postprocessor,
            last_urdf_path=self.last_urdf_path,
            base_x=self.base_x,
            base_y=self.base_y,
            base_z=self.base_z,
            base_a=self.base_a,
            base_b=self.base_b,
            base_c=self.base_c,
            tool_x=self.tool_x,
            tool_y=self.tool_y,
            tool_z=self.tool_z,
            tool_a=self.tool_a,
            tool_b=self.tool_b,
            tool_c=self.tool_c,
            table_x1=self.table_x1,
            table_y1=self.table_y1,
            table_x2=self.table_x2,
            table_y2=self.table_y2,
            show_table=self.show_table,
            show_robot=self.show_robot,
            ik_config=self.ik_config or DEFAULT_IK_CONFIG,
        )

    def _load_meta(self):
        """Read viewer_settings.json to determine which project to open."""
        self._project_file = self._project_store.load_last_project_path()
        self._project_name = self._project_store.project_name_from_path(self._project_file)

    def _save_meta(self):
        """Write the current project name to viewer_settings.json."""
        try:
            self._project_store.save_last_project_path(self._project_file)
        except Exception as exc:
            print(f"Error saving meta: {exc}")

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
        except Exception as exc:
            print(f"Error saving project: {exc}")

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
