import os
import subprocess
import sys

from PyQt6.QtCore import QProcess
from PyQt6.QtWidgets import QFileDialog, QMessageBox, QInputDialog

from .app_paths import CSV_DIR, GCODE_DIR, OUTPUT_DIR, PACKAGE_DIR, REPO_ROOT, ensure_directory
from .error_reporting import get_logger
from .plugin_registry import discover_python_plugins
from .robot_catalog import RobotLoadThread, discover_robot_descriptors
from .viewer_components import DataLoaderThread, ImporterThread


logger = get_logger(__name__)


class ViewerIOMixin:
    def _discover_plugins(self, relative_dir, required_attrs=()):
        """Discover Python plugins relative to the ROOT application directory."""
        plugin_dir = PACKAGE_DIR / relative_dir
        return discover_python_plugins(plugin_dir, REPO_ROOT, required_attrs=required_attrs)

    def _select_plugin_by_file_name(self, combo, file_name):
        """Select combobox item by plugin file name, or clear the selection."""
        target_index = -1
        if file_name:
            for idx in range(combo.count()):
                spec = combo.itemData(idx)
                if spec is not None and getattr(spec, "file_name", None) == file_name:
                    target_index = idx
                    break
        combo.setCurrentIndex(target_index)

    def _capture_file_state(self, file_path):
        if not os.path.exists(file_path):
            return None
        return (os.path.getmtime(file_path), os.path.getsize(file_path))

    def _file_was_updated(self, file_path, previous_state):
        current_state = self._capture_file_state(file_path)
        if current_state is None:
            return False
        return previous_state is None or current_state != previous_state

    def _open_local_file(self, file_path):
        if os.name == 'nt':
            os.startfile(file_path)
            return

        import platform
        if platform.system() == 'Darwin':
            subprocess.call(('open', file_path))
        else:
            subprocess.call(('xdg-open', file_path))

    def _run_postprocessor_job(
        self,
        plugin_spec,
        output_file,
        status_text,
        success_text,
        failure_status,
        failure_message,
        success_callback=None,
    ):
        previous_state = self._capture_file_state(output_file)
        ticket = self._begin_job("postprocess", status_text)

        def on_finished(exit_code, exit_status):
            del exit_status
            if not self.task_controller.is_current(ticket):
                return
            self._end_job(ticket)
            if exit_code == 0 and self._file_was_updated(output_file, previous_state):
                self.lbl_status.setText(success_text)
                if success_callback is not None:
                    success_callback(output_file)
                return

            logger.error("Postprocessor %s did not produce expected output %s", plugin_spec.file_name, output_file)
            QMessageBox.critical(self, "Postprocessor Error", failure_message)
            self.lbl_status.setText(failure_status)

        self._run_postprocessor_async(plugin_spec, self.last_file_path, output_file, on_finished)

    def populate_postprocessors(self):
        """Scan the Postprocesor directory and populate the combobox."""
        self.combo_postprocessor.clear()
        self._postprocessor_specs = self._discover_plugins("Postprocesor", required_attrs=("run_export",))
        for spec in self._postprocessor_specs:
            self.combo_postprocessor.addItem(spec.display_name, userData=spec)
        self._select_plugin_by_file_name(self.combo_postprocessor, self.last_postprocessor)

    def _run_postprocessor_async(self, plugin_spec, input_file, output_file, on_finished):
        """Run a postprocessor script as a non-blocking QProcess."""
        self._qprocess = QProcess(self)
        self._qprocess.setWorkingDirectory(str(REPO_ROOT))
        self._qprocess.finished.connect(on_finished)
        self._qprocess.start(sys.executable, ["-m", plugin_spec.module_name, input_file, output_file])

    def on_preview_clicked(self):
        """Run the selected postprocessor and open the preview file."""
        plugin_spec = self.combo_postprocessor.currentData()
        if not plugin_spec:
            QMessageBox.warning(self, "Warning", "Please select a postprocessor first.")
            return

        if not self.last_file_path or not os.path.exists(self.last_file_path):
            QMessageBox.warning(self, "Warning", "Please load a CSV file first.")
            return

        preview_file = str(ensure_directory(OUTPUT_DIR) / "preview.txt")
        self._run_postprocessor_job(
            plugin_spec,
            preview_file,
            status_text=f"Running preview with {plugin_spec.file_name}...",
            success_text="Preview generated successfully.",
            failure_status="Preview failed.",
            failure_message="Postprocessor finished without producing a new preview file.",
            success_callback=self._open_local_file,
        )

    def on_export_clicked(self):
        """Run the selected postprocessor and save to a user-chosen location."""
        plugin_spec = self.combo_postprocessor.currentData()
        if not plugin_spec:
            QMessageBox.warning(self, "Warning", "Please select a postprocessor first.")
            return

        if not self.last_file_path or not os.path.exists(self.last_file_path):
            QMessageBox.warning(self, "Warning", "Please load a CSV file first.")
            return

        out_file, _ = QFileDialog.getSaveFileName(self, "Export File As", "", "Text Files (*.txt);;All Files (*)")
        if not out_file:
            return

        self._run_postprocessor_job(
            plugin_spec,
            out_file,
            status_text=f"Exporting with {plugin_spec.file_name}...",
            success_text=f"Exported successfully to {os.path.basename(out_file)}.",
            failure_status="Export failed.",
            failure_message="Postprocessor finished without writing the output file.",
            success_callback=lambda path: QMessageBox.information(
                self,
                "Success",
                f"File exported successfully to:\n{path}",
            ),
        )

    def populate_importers(self):
        """Scan Importer/ dir, dynamically load each plugin, fill combobox."""
        self.combo_importer.clear()
        self._importer_specs = []

        self._importer_specs = self._discover_plugins("Importer", required_attrs=("run_import",))
        for spec in self._importer_specs:
            self.combo_importer.addItem(spec.display_name, userData=spec)

        self.btn_run_import.setEnabled(bool(self._importer_specs) and bool(self._import_gcode_path))

    def _select_gcode_for_import(self):
        """Open GCODE/ dialog, call each plugin's can_handle() to pre-select."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select G-Code File",
            str(ensure_directory(GCODE_DIR)),
            "G-Code Files (*.gcode *.gco *.nc);;All Files (*)",
        )
        if not file_path:
            return

        self._import_gcode_path = file_path
        self.lbl_import_file.setText(os.path.basename(file_path))

        matched = -1
        for i, spec in enumerate(self._importer_specs):
            try:
                if hasattr(spec.module, 'can_handle') and spec.module.can_handle(file_path):
                    matched = i
                    break
            except Exception:
                logger.exception("Importer auto-detection failed for %s", getattr(spec, "file_name", "<unknown>"))
                continue

        if matched >= 0:
            self.combo_importer.setCurrentIndex(matched)
            self.lbl_import_status.setText(
                f"Auto-detected: '{self.combo_importer.itemText(matched)}' selected. "
                f"Override if needed, then click Import & Load."
            )
        else:
            self.lbl_import_status.setText(
                "Auto-detection found no match. Select an importer manually."
            )

        self.btn_run_import.setEnabled(bool(self._importer_specs))

    def _run_import(self):
        """Invoke the selected plugin's run_import() in a background thread."""
        if not self._import_gcode_path:
            QMessageBox.warning(self, "No File", "Select a G-code file first.")
            return

        plugin_spec = self.combo_importer.currentData()
        if not plugin_spec:
            QMessageBox.warning(self, "No Importer", "No importer selected.")
            return

        mod = plugin_spec.module
        fname = plugin_spec.file_name

        name_only, _ = os.path.splitext(os.path.basename(self._import_gcode_path))
        csv_out_path = str(ensure_directory(CSV_DIR) / f"{name_only}.csv")

        self.lbl_import_status.setText(f"Running {fname}...")
        ticket = self._begin_job("import", f"Running {fname}...")

        self._import_thread = ImporterThread(mod, self._import_gcode_path, csv_out_path)
        self._import_thread.finished_signal.connect(
            lambda success, payload, tk=ticket: self._on_import_finished(tk, success, payload)
        )
        self._import_thread.start()

    def _on_import_finished(self, ticket, success: bool, payload: str):
        """Handle ImporterThread completion."""
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        if success:
            self.lbl_import_status.setText("Import done. Loading CSV...")
            self.lbl_status.setText("Import successful.")
            self.load_file(payload)
        else:
            QMessageBox.critical(self, "Import Error", f"Importer raised an error:\n\n{payload}")
            self.lbl_import_status.setText("Import failed.")
            self.lbl_status.setText("Import failed.")

    def load_file_dialog(self):
        """Open a file dialog to select and load a CSV file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select CSV Data Stream",
            str(ensure_directory(CSV_DIR)),
            "CSV Files (*.csv);;All Files (*)",
        )
        if not file_path:
            return
        self.load_file(file_path)

    def load_urdf_dialog(self):
        """Scan for URDF files and show a selection dialog."""
        self._robot_descriptors = discover_robot_descriptors(str(PACKAGE_DIR))
        if not self._robot_descriptors:
            QMessageBox.information(
                self,
                "No Robots Found",
                "No URDF files found in subdirectories of ROOT.",
            )
            return

        labels = [descriptor.label for descriptor in self._robot_descriptors]
        current = 0
        if self.last_urdf_path:
            for i, descriptor in enumerate(self._robot_descriptors):
                if os.path.normpath(descriptor.file_path) == os.path.normpath(self.last_urdf_path):
                    current = i
                    break

        chosen, ok = QInputDialog.getItem(
            self,
            "Select Robot",
            "Available robots:",
            labels,
            current,
            False,
        )
        if not ok:
            return

        idx = labels.index(chosen)
        self.load_urdf(self._robot_descriptors[idx].file_path)

    def load_urdf(self, file_path):
        """Load a URDF robot model and add its meshes to the scene."""
        if not os.path.exists(file_path):
            logger.error("URDF file not found: %s", file_path)
            self.lbl_status.setText("URDF file not found.")
            return

        ticket = self._begin_job("robot_load", f"Loading Robot: {os.path.basename(file_path)}...")
        self._robot_thread = RobotLoadThread(self.robot_simulator_cls, file_path)
        self._robot_thread.finished_signal.connect(
            lambda success, simulator, robot_display, error, tk=ticket, fpath=file_path:
                self._on_robot_load_finished(tk, fpath, success, simulator, robot_display, error)
        )
        self._robot_thread.start()

    def _on_robot_load_finished(self, ticket, file_path, success, simulator, robot_display, error):
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)

        if not success:
            logger.error("Robot load failed for %s: %s", file_path, error)
            QMessageBox.critical(self, "Error Loading Robot", f"Failed to load URDF:\n{error}")
            self.lbl_status.setText("Robot load failed.")
            self._restore_interaction_state()
            return

        self.last_urdf_path = file_path
        self.robot_sim = simulator
        self._reset_ik_tracking()

        for actor in self.robot_actors.values():
            self.plotter.remove_actor(actor)
        self.robot_actors.clear()

        for link_name, mesh in self.robot_sim.link_meshes.items():
            actor = self.plotter.add_mesh(mesh, color="orange", show_edges=True)
            actor.SetVisibility(self.show_robot)
            self.robot_actors[link_name] = actor

        self.lbl_status.setText("Robot loaded successfully.")
        self.lbl_robot_name.setText(f"\U0001F916 {robot_display}")
        self.plotter.reset_camera()

        if self.points_xyz is not None and len(self.points_xyz) > 0:
            self.color_strip.set_statuses(None)

        self.update_plot()
        self._restore_interaction_state()

    def load_file(self, file_path):
        """Start asynchronous loading of a CSV trajectory file."""
        if not os.path.exists(file_path):
            logger.error("CSV file not found: %s", file_path)
            self.lbl_status.setText("Last saved file not found.")
            return

        self.last_file_path = file_path
        ticket = self._begin_job("load", f"Loading {os.path.basename(file_path)}...")

        self.loader_thread = DataLoaderThread(file_path)
        self.loader_thread.finished_signal.connect(
            lambda points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer, estimated_time_s, estimated_weight_g, tk=ticket:
                self.on_load_finished(
                    tk,
                    points_xyz,
                    orientations_abc,
                    colors_rgb,
                    layer_ends,
                    max_layer,
                    estimated_time_s,
                    estimated_weight_g,
                )
        )
        self.loader_thread.error_signal.connect(lambda err_msg, tk=ticket: self.on_load_error(tk, err_msg))
        self.loader_thread.start()

    def on_load_error(self, ticket, err_msg):
        """Handle CSV loading errors."""
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        logger.error("CSV load failed: %s", err_msg)
        QMessageBox.critical(self, "Error Loading File", err_msg)
        self.lbl_status.setText("Load failed.")

    def _configure_loaded_trajectory(self, points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer):
        self.points_xyz = points_xyz
        self._original_points_xyz = points_xyz.copy()
        self._original_orientations_abc = orientations_abc.copy()
        self.orientations_abc = orientations_abc
        self.colors_rgb = colors_rgb
        self.layer_end_indices = layer_ends
        self.max_layer = max_layer
        self.trajectory_renderer.set_trajectory(self.points_xyz, self.colors_rgb, self.print_thickness)
        self._sorted_layer_ends = sorted(layer_ends.items(), key=lambda kv: kv[1])

    def _reset_edit_controls(self):
        self.spin_edit_x.setValue(0.0)
        self.spin_edit_y.setValue(0.0)
        self.spin_edit_z_rot.setValue(0.0)
        self.lbl_edit_status.setText("")

    def _configure_progress_controls(self, num_pts, max_layer):
        self.updating_sliders = True
        self.h_slider.setMinimum(0)
        self.h_slider.setMaximum(num_pts)
        self.h_slider.setValue(num_pts)
        self.h_slider.setEnabled(True)
        self.v_slider.setMinimum(0)
        self.v_slider.setMaximum(max_layer)
        self.v_slider.setValue(max_layer)
        self.v_slider.setEnabled(True)
        self.updating_sliders = False

    def on_load_finished(
        self,
        ticket,
        points_xyz,
        orientations_abc,
        colors_rgb,
        layer_ends,
        max_layer,
        estimated_time_s,
        estimated_weight_g,
    ):
        """Handle successful CSV load: store data and update UI."""
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        if points_xyz is None or len(points_xyz) == 0:
            QMessageBox.warning(self, "No Data", "No valid points found in CSV.")
            self.lbl_status.setText("Load failed.")
            return

        self._configure_loaded_trajectory(points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer)
        self._reset_edit_controls()

        num_pts = len(self.points_xyz)
        self._configure_progress_controls(num_pts, max_layer)

        self.current_step = num_pts
        self.max_layer = max_layer
        self.estimated_time_s = estimated_time_s
        self.estimated_weight_g = estimated_weight_g
        self._reset_ik_tracking()
        self.current_layer = max_layer

        self.update_ui_labels()
        self.update_plot()
        self.plotter.reset_camera()

        hours = int(estimated_time_s // 3600)
        minutes = int((estimated_time_s % 3600) // 60)
        seconds = int(estimated_time_s % 60)
        if hours > 0:
            time_str = f"{hours}h {minutes:02d}m {seconds:02d}s"
        else:
            time_str = f"{minutes}m {seconds:02d}s"
        self.lbl_print_time.setText(f"Estimated print time: {time_str}")
        self.lbl_print_weight.setText(f"Estimated weight: {estimated_weight_g:.1f} g")

        self.lbl_status.setText(f"Loaded {num_pts} points.")
        self.color_strip.set_statuses(None)
        self._restore_interaction_state()
