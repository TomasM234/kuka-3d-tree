import os
import subprocess
import sys
import multiprocessing
import math
import numpy as np
# pyvista and pyvistaqt are imported lazily in __main__ (after splash is visible)
# because they take several seconds to load (VTK initialization).
pv = None
QtInteractor = None
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSlider, QFileDialog, QMessageBox, QFrame, QSizePolicy, QComboBox,
    QDoubleSpinBox, QFormLayout, QTabWidget, QCheckBox, QSplashScreen
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QRectF, QTimer, QProcess
from PyQt6.QtGui import QPainter, QColor
from plugin_registry import discover_python_plugins
from pose_math import kuka_base_to_matrix, matrix_to_kuka_abc
from project_config import DEFAULT_IK_CONFIG, ProjectConfig, ProjectConfigStore
from robot_catalog import RobotLoadThread, discover_robot_descriptors
from task_controller import TaskController
from trajectory_render import TrajectoryRenderer
from trajectory_schema import (
    apply_planar_edit_transform,
    load_trajectory_csv,
    rewrite_trajectory_csv,
)
from trajectory_test_lib import TrajectoryTestConfig, run_trajectory_test_parallel

# robot_ik (ikpy / yourdfpy / trimesh) is also imported lazily in __main__.
RobotSimulator = None


# ---------------------------------------------------------------------------
# Predefined Initial Robot Configurations (IK Seeds in Radians)
# Values represent joint angles 1 to 6
# ---------------------------------------------------------------------------
IK_CONFIGS = {
    "Default (Zero)":       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "Elbow Up / Front":     [0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
    "Elbow Down / Front":   [0.0, 0.0, -1.57, 0.0, -1.57, 0.0],
    "Elbow Up / Back":      [3.14, -1.57, 1.57, 0.0, 1.57, 0.0],
    "Elbow Down / Back":    [3.14, 0.0, -1.57, 0.0, -1.57, 0.0],
    "Transport":            [0.0, -2.0, 2.0, 0.0, 2.0, 0.0]
}


class ColorStripWidget(QWidget):
    """Thin horizontal bar showing trajectory test results as colored segments."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(12)
        self.statuses = None
        self.colors = {
            0: QColor(0, 220, 0),    # OK (Green)
            1: QColor(255, 200, 0),  # Singularity (Yellow)
            2: QColor(255, 0, 0),    # Limit (Red)
            3: QColor(100, 100, 100) # Unreachable (Grey)
        }

    def set_statuses(self, statuses):
        """Set the status array and trigger a repaint."""
        self.statuses = statuses
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        painter.fillRect(rect, QColor(220, 220, 220))

        if self.statuses is None or len(self.statuses) == 0:
            return

        n = len(self.statuses)
        w = rect.width()
        h = rect.height()

        current_status = self.statuses[0]
        start_idx = 0

        for i in range(1, n + 1):
            if i == n or self.statuses[i] != current_status:
                x_start = (start_idx / n) * w
                x_end = (i / n) * w
                seg_w = x_end - x_start
                painter.fillRect(QRectF(x_start, 0, seg_w, h), self.colors.get(current_status, QColor(0, 0, 0)))

                if i < n:
                    current_status = self.statuses[i]
                    start_idx = i


class TrajectoryTestThread(QThread):
    """Background thread that runs trajectory testing via trajectory_test_lib."""

    progress_signal = pyqtSignal(int, int)   # current, total
    finished_signal = pyqtSignal(object)     # numpy array of statuses

    def __init__(self, points_xyz, orientations_abc, robot_sim, base_params, tool_params, seed_templates):
        super().__init__()
        self.points_xyz = points_xyz
        self.orientations_abc = orientations_abc
        self.urdf_path = robot_sim.urdf_path
        self.has_model = robot_sim.urdf_model is not None
        self.base_params = base_params
        self.tool_params = tool_params
        self.seed_templates = tuple(np.array(seed, dtype=np.float64, copy=True) for seed in seed_templates)

    def run(self):
        if not self.has_model:
            self.finished_signal.emit(None)
            return

        try:
            config = TrajectoryTestConfig(
                urdf_path=self.urdf_path,
                base_params=self.base_params,
                tool_params=self.tool_params,
                seed_templates=self.seed_templates
            )
            statuses = run_trajectory_test_parallel(
                self.points_xyz,
                self.orientations_abc,
                config,
                progress_callback=lambda current, total: self.progress_signal.emit(current, total)
            )
        except Exception as e:
            print(f"Trajectory test error: {e}")
            self.finished_signal.emit(None)
            return

        self.finished_signal.emit(statuses)


class DataLoaderThread(QThread):
    """Background thread that parses a CSV trajectory file into numpy arrays."""
    # points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer, estimated_time_s, estimated_weight_g
    finished_signal = pyqtSignal(object, object, object, object, int, float, float)
    error_signal = pyqtSignal(str)

    def __init__(self, file_path):
        super().__init__()
        self.file_path = file_path

    def run(self):
        """Parse CSV via the shared trajectory schema helpers."""
        try:
            data = load_trajectory_csv(self.file_path)
            self.finished_signal.emit(
                data.points_xyz,
                data.orientations_abc,
                data.colors_rgb,
                data.layer_end_indices,
                data.max_layer,
                data.estimated_time_s,
                data.estimated_weight_g,
            )
        except Exception as e:
            self.error_signal.emit(str(e))


class ImporterThread(QThread):
    """Background thread that calls a plugin importer's run_import() function.

    Signals:
        finished_signal(success: bool, payload: str)
            On success: payload = path to the output CSV.
            On failure: payload = human-readable error message.
    """

    finished_signal = pyqtSignal(bool, str)

    def __init__(self, importer_module, input_path: str, output_csv: str):
        super().__init__()
        self.importer_module = importer_module
        self.input_path = input_path
        self.output_csv = output_csv

    def run(self):
        try:
            self.importer_module.run_import(self.input_path, self.output_csv)
            self.finished_signal.emit(True, self.output_csv)
        except Exception as exc:
            self.finished_signal.emit(False, str(exc))


class RobotPathViewer(QMainWindow):
    """Main application window: 3D trajectory viewer for KUKA robot CSV data streams."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot CSV 3D Viewer (High-Performance PyVista Edition)")
        self.resize(1300, 900)

        app_dir = os.path.dirname(os.path.abspath(__file__))
        self._meta_file = os.path.join(app_dir, "viewer_settings.json")
        self._project_dir = os.path.join(app_dir, "Project")
        self._project_store = ProjectConfigStore(self._meta_file, self._project_dir)
        self._project_file = ""  # resolved by _load_meta()
        self._project_name = "default"

        default_project = ProjectConfig()

        # Data state
        self.points_xyz = None
        self._original_points_xyz = None
        self._original_orientations_abc = None
        self.orientations_abc = None
        self.colors_rgb = None
        self.layer_end_indices = {}
        self._sorted_layer_ends = []  # [(layer_idx, end_point), ...] sorted by end_point
        self.max_layer = 0
        self.current_step = 0
        self.current_layer = 0
        self.estimated_time_s = 0.0
        self.estimated_weight_g = 0.0
        self._apply_project_config_to_state(default_project)

        # Table state
        self.table_actor = None

        # Robot simulator
        self.robot_sim = RobotSimulator()
        self.robot_actors = {}

        # Importer plugin modules loaded from Importer/ directory
        self._importer_specs = []
        self._postprocessor_specs = []
        self._robot_descriptors = []
        self._import_gcode_path = ""  # last selected G-code file

        self.updating_sliders = False
        self.task_controller = TaskController(self._on_busy_changed, self._set_status_text)
        self._load_ticket = None
        self._import_ticket = None
        self._robot_ticket = None
        self._traj_ticket = None
        self._postprocess_ticket = None
        
        # IK state: keep the previous solution for continuity and only
        # re-apply the selected preset when the user explicitly changes it.
        self.last_ik_solution = None
        self._force_ik_seed_from_config = False

        # Debounce timer for slider-driven updates
        self._render_timer = QTimer()
        self._render_timer.setSingleShot(True)
        self._render_timer.setInterval(60)
        self._render_timer.timeout.connect(self._deferred_update_plot)
        self._pending_tube_render = False

        # Build UI
        self._create_main_layout()
        self._create_general_tab()
        self._create_workplace_tab()
        self._create_edit_tab()
        self._create_export_tab()
        self._create_viewport()
        self.trajectory_renderer = TrajectoryRenderer(self.plotter, pv)

        self.setup_scene()

    # ------------------------------------------------------------------
    # UI Construction helpers
    # ------------------------------------------------------------------

    def _create_main_layout(self):
        """Create the top-level horizontal split: left panel + right viewport."""
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)

        self.left_panel = QFrame()
        self.left_panel.setFixedWidth(250)
        self.left_layout = QVBoxLayout(self.left_panel)
        self.left_panel.setFrameShape(QFrame.Shape.StyledPanel)
        self.main_layout.addWidget(self.left_panel)

        self.tabs = QTabWidget()
        self.tabs.tabBar().setUsesScrollButtons(False)
        self.tabs.tabBar().setExpanding(True)
        self.left_layout.addWidget(self.tabs)

        # ---- CURRENT SYSTEM POSE -----------------------------------------
        self.left_layout.addSpacing(10)
        self.left_layout.addWidget(QLabel("<b>CURRENT ROBOT POSE</b>"))
        
        self.pose_grid = QVBoxLayout()
        self.pose_grid.setContentsMargins(5, 5, 5, 5)
        self.pose_grid.setSpacing(2)
        
        self.lbl_pose_xyz = QLabel("TCP mm: X:- Y:- Z:-")
        self.lbl_pose_abc = QLabel("TCP deg: A:- B:- C:-")
        self.lbl_pose_j123 = QLabel("J1-3 deg: A1:- A2:- A3:-")
        self.lbl_pose_j456 = QLabel("J4-6 deg: A4:- A5:- A6:-")
        
        for lbl in [self.lbl_pose_xyz, self.lbl_pose_abc, self.lbl_pose_j123, self.lbl_pose_j456]:
            lbl.setStyleSheet("font-family: monospace; font-size: 11px;")
            lbl.setWordWrap(False)
            lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            lbl.setFixedHeight(lbl.fontMetrics().height() + 6)
            lbl.setToolTip(lbl.text())
            
        self.pose_grid.addWidget(self.lbl_pose_xyz)
        self.pose_grid.addWidget(self.lbl_pose_abc)
        self.pose_grid.addWidget(self.lbl_pose_j123)
        self.pose_grid.addWidget(self.lbl_pose_j456)
        
        self.left_layout.addLayout(self.pose_grid)

    def _set_status_text(self, text):
        self.lbl_status.setText(text)

    def _on_busy_changed(self, busy):
        self.left_panel.setEnabled(not busy)
        self.h_slider.setEnabled(not busy and self.points_xyz is not None and len(self.points_xyz) > 0)
        self.v_slider.setEnabled(not busy and self.points_xyz is not None and len(self.points_xyz) > 0)
        self.btn_test_traj.setEnabled(
            not busy and
            self.points_xyz is not None and len(self.points_xyz) > 0 and
            self.robot_sim.urdf_model is not None
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
        self.robot_sim = RobotSimulator()
        self.lbl_robot_name.setText("No robot selected.")
        self._reset_pose_labels()
        self._reset_ik_tracking()

    def _discover_plugins(self, relative_dir, required_attrs=()):
        """Discover Python plugins relative to the ROOT application directory."""
        app_dir = os.path.dirname(os.path.abspath(__file__))
        plugin_dir = os.path.join(app_dir, relative_dir)
        return discover_python_plugins(plugin_dir, app_dir, required_attrs=required_attrs)

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

    def _create_spinbox(self, min_val, max_val, suffix, value, callback):
        """Helper to create a configured QDoubleSpinBox and connect its signal."""
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setSuffix(suffix)
        spin.setValue(value)
        spin.valueChanged.connect(callback)
        return spin

    def _create_general_tab(self):
        """Build the 'General' tab: CSV loading, G-code import, robot, camera, visuals."""
        self.tab_general = QWidget()
        self.layout_general = QVBoxLayout(self.tab_general)
        self.tabs.addTab(self.tab_general, "General")

        # ---- PROJECT ----------------------------------------------------
        self.layout_general.addWidget(QLabel("<b>PROJECT</b>"))

        self.lbl_project_name = QLabel("\U0001F4C1 default")
        self.lbl_project_name.setWordWrap(True)
        self.layout_general.addWidget(self.lbl_project_name)

        proj_btn_row = QHBoxLayout()
        btn_new_proj = QPushButton("New")
        btn_new_proj.clicked.connect(self._new_project)
        proj_btn_row.addWidget(btn_new_proj)
        btn_open_proj = QPushButton("Open")
        btn_open_proj.clicked.connect(self._open_project)
        proj_btn_row.addWidget(btn_open_proj)
        btn_saveas_proj = QPushButton("Save As...")
        btn_saveas_proj.clicked.connect(self._save_project_as)
        proj_btn_row.addWidget(btn_saveas_proj)
        self.layout_general.addLayout(proj_btn_row)

        self.layout_general.addSpacing(12)

        # ---- TRAJECTORY (CSV) -------------------------------------------
        self.layout_general.addWidget(QLabel("<b>TRAJECTORY (CSV)</b>"))

        self.btn_load = QPushButton("Open CSV File...")
        self.btn_load.clicked.connect(self.load_file_dialog)
        self.layout_general.addWidget(self.btn_load)

        self.layout_general.addSpacing(12)

        # ---- IMPORT FROM G-CODE -----------------------------------------
        self.layout_general.addWidget(QLabel("<b>IMPORT FROM G-CODE</b>"))

        self.lbl_import_file = QLabel("No file selected.")
        self.lbl_import_file.setWordWrap(True)
        self.layout_general.addWidget(self.lbl_import_file)

        self.btn_select_gcode = QPushButton("Select G-Code File...")
        self.btn_select_gcode.clicked.connect(self._select_gcode_for_import)
        self.layout_general.addWidget(self.btn_select_gcode)

        self.layout_general.addWidget(QLabel("Importer:"))
        self.combo_importer = QComboBox()
        self.layout_general.addWidget(self.combo_importer)

        self.btn_run_import = QPushButton("Import && Load")
        self.btn_run_import.clicked.connect(self._run_import)
        self.btn_run_import.setEnabled(False)
        self.layout_general.addWidget(self.btn_run_import)

        self.lbl_import_status = QLabel("")
        self.lbl_import_status.setWordWrap(True)
        self.layout_general.addWidget(self.lbl_import_status)

        self.populate_importers()

        self.layout_general.addSpacing(12)

        # ---- STATUS -----------------------------------------------------
        self.lbl_status = QLabel("Ready.")
        self.lbl_status.setWordWrap(True)
        self.layout_general.addWidget(self.lbl_status)

        self.lbl_print_time = QLabel("")
        self.layout_general.addWidget(self.lbl_print_time)

        self.lbl_print_weight = QLabel("")
        self.layout_general.addWidget(self.lbl_print_weight)

        self.layout_general.addSpacing(12)

        # ---- CAMERA -----------------------------------------------------
        self.layout_general.addWidget(QLabel("<b>CAMERA</b>"))

        for label, slot in [("Reset View (Iso)", self.view_isometric),
                            ("Top View", self.view_top),
                            ("Side View", self.view_side)]:
            btn = QPushButton(label)
            btn.clicked.connect(slot)
            self.layout_general.addWidget(btn)

        self.layout_general.addSpacing(12)

        # ---- VISUALS ----------------------------------------------------
        self.layout_general.addWidget(QLabel("<b>VISUALS</b>"))

        self.lbl_thickness = QLabel(f"Extrusion Width:\n{self.print_thickness:.1f} mm")
        self.layout_general.addWidget(self.lbl_thickness)

        self.slider_thick = QSlider(Qt.Orientation.Horizontal)
        self.slider_thick.setMinimum(2)
        self.slider_thick.setMaximum(50)
        self.slider_thick.setValue(int(self.print_thickness * 10))
        self.slider_thick.valueChanged.connect(self.on_thickness_changed)
        self.layout_general.addWidget(self.slider_thick)
        self.layout_general.addStretch()

    def _create_workplace_tab(self):
        """Build the 'Workplace' tab: table, robot base/tool transforms."""
        self.tab_workplace = QWidget()
        self.layout_workplace = QVBoxLayout(self.tab_workplace)
        self.tabs.addTab(self.tab_workplace, "Workplace")

        # Table section
        self.layout_workplace.addWidget(QLabel("<b>TABLE (Bed)</b>"))

        self.check_show_table = QCheckBox("Show Table")
        self.check_show_table.setChecked(self.show_table)
        self.check_show_table.stateChanged.connect(self.on_table_changed)
        self.layout_workplace.addWidget(self.check_show_table)

        self.table_form = QFormLayout()
        self.spin_table_x1 = self._create_spinbox(-5000, 5000, " mm", self.table_x1, self.on_table_changed)
        self.spin_table_y1 = self._create_spinbox(-5000, 5000, " mm", self.table_y1, self.on_table_changed)
        self.spin_table_x2 = self._create_spinbox(-5000, 5000, " mm", self.table_x2, self.on_table_changed)
        self.spin_table_y2 = self._create_spinbox(-5000, 5000, " mm", self.table_y2, self.on_table_changed)
        self.table_form.addRow("X1 (Corner 1):", self.spin_table_x1)
        self.table_form.addRow("Y1 (Corner 1):", self.spin_table_y1)
        self.table_form.addRow("X2 (Corner 2):", self.spin_table_x2)
        self.table_form.addRow("Y2 (Corner 2):", self.spin_table_y2)
        self.layout_workplace.addLayout(self.table_form)
        self.layout_workplace.addSpacing(15)

        # Robot section
        self.layout_workplace.addWidget(QLabel("<b>ROBOT</b>"))

        self.lbl_robot_name = QLabel("No robot selected.")
        self.lbl_robot_name.setWordWrap(True)
        self.layout_workplace.addWidget(self.lbl_robot_name)

        self.btn_load_urdf = QPushButton("Select Robot...")
        self.btn_load_urdf.clicked.connect(self.load_urdf_dialog)
        self.layout_workplace.addWidget(self.btn_load_urdf)

        self.btn_test_traj = QPushButton("Test Trajectory")
        self.btn_test_traj.clicked.connect(self.run_trajectory_test)
        self.btn_test_traj.setEnabled(False)
        self.layout_workplace.addWidget(self.btn_test_traj)

        self.layout_workplace.addSpacing(5)

        # IK Start Configuration
        self.layout_workplace.addWidget(QLabel("<b>IK Start Configuration:</b>"))
        self.combo_ik_config = QComboBox()
        self.combo_ik_config.addItems(list(IK_CONFIGS.keys()))
        self.combo_ik_config.currentTextChanged.connect(self.on_ik_config_changed)
        self.layout_workplace.addWidget(self.combo_ik_config)

        self.layout_workplace.addSpacing(10)

        self.check_show_robot = QCheckBox("Show Robot")
        self.check_show_robot.setChecked(self.show_robot)
        self.check_show_robot.stateChanged.connect(self.on_robot_changed)
        self.layout_workplace.addWidget(self.check_show_robot)

        # Base frame
        self.layout_workplace.addWidget(QLabel("<u>Robot Base Frame:</u>"))
        self.base_form = QFormLayout()
        self.spin_base_x = self._create_spinbox(-5000, 5000, " mm", self.base_x, self.on_transform_changed)
        self.spin_base_y = self._create_spinbox(-5000, 5000, " mm", self.base_y, self.on_transform_changed)
        self.spin_base_z = self._create_spinbox(-5000, 5000, " mm", self.base_z, self.on_transform_changed)
        self.spin_base_a = self._create_spinbox(-360, 360, " Â°", self.base_a, self.on_transform_changed)
        self.spin_base_b = self._create_spinbox(-360, 360, " Â°", self.base_b, self.on_transform_changed)
        self.spin_base_c = self._create_spinbox(-360, 360, " Â°", self.base_c, self.on_transform_changed)
        for label, spin in [("X:", self.spin_base_x), ("Y:", self.spin_base_y), ("Z:", self.spin_base_z),
                            ("A:", self.spin_base_a), ("B:", self.spin_base_b), ("C:", self.spin_base_c)]:
            self.base_form.addRow(label, spin)
        self.layout_workplace.addLayout(self.base_form)
        self.layout_workplace.addSpacing(10)

        # Tool frame
        self.layout_workplace.addWidget(QLabel("<u>Robot Tool Frame:</u>"))
        self.tool_form = QFormLayout()
        self.spin_tool_x = self._create_spinbox(-5000, 5000, " mm", self.tool_x, self.on_transform_changed)
        self.spin_tool_y = self._create_spinbox(-5000, 5000, " mm", self.tool_y, self.on_transform_changed)
        self.spin_tool_z = self._create_spinbox(-5000, 5000, " mm", self.tool_z, self.on_transform_changed)
        self.spin_tool_a = self._create_spinbox(-360, 360, " Â°", self.tool_a, self.on_transform_changed)
        self.spin_tool_b = self._create_spinbox(-360, 360, " Â°", self.tool_b, self.on_transform_changed)
        self.spin_tool_c = self._create_spinbox(-360, 360, " Â°", self.tool_c, self.on_transform_changed)
        for label, spin in [("X:", self.spin_tool_x), ("Y:", self.spin_tool_y), ("Z:", self.spin_tool_z),
                            ("A:", self.spin_tool_a), ("B:", self.spin_tool_b), ("C:", self.spin_tool_c)]:
            self.tool_form.addRow(label, spin)
        self.layout_workplace.addLayout(self.tool_form)
        self.layout_workplace.addStretch()

    def _create_edit_tab(self):
        """Build the 'Edit' tab: translate X/Y and rotate around Z."""
        self.tab_edit = QWidget()
        self.layout_edit = QVBoxLayout(self.tab_edit)
        self.tabs.addTab(self.tab_edit, "Edit")

        self.layout_edit.addWidget(QLabel("<b>TRANSFORM MODEL</b>"))
        self.layout_edit.addWidget(QLabel(
            "Total offset from original data.\n"
            "Rotation is around the model centroid."))
        self.layout_edit.addSpacing(10)

        # --- Shift X ---
        self.layout_edit.addWidget(QLabel("<u>Shift X:</u>"))
        self.spin_edit_x = self._create_spinbox(-10000, 10000, " mm", 0.0, lambda: None)
        self.layout_edit.addWidget(self.spin_edit_x)
        hx = QHBoxLayout()
        for label, delta in [("-100", -100), ("-1", -1), ("+1", 1), ("+100", 100)]:
            b = QPushButton(label)
            b.setFixedWidth(50)
            b.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_x, d))
            hx.addWidget(b)
        self.layout_edit.addLayout(hx)
        self.layout_edit.addSpacing(5)

        # --- Shift Y ---
        self.layout_edit.addWidget(QLabel("<u>Shift Y:</u>"))
        self.spin_edit_y = self._create_spinbox(-10000, 10000, " mm", 0.0, lambda: None)
        self.layout_edit.addWidget(self.spin_edit_y)
        hy = QHBoxLayout()
        for label, delta in [("-100", -100), ("-1", -1), ("+1", 1), ("+100", 100)]:
            b = QPushButton(label)
            b.setFixedWidth(50)
            b.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_y, d))
            hy.addWidget(b)
        self.layout_edit.addLayout(hy)
        self.layout_edit.addSpacing(5)

        # --- Rotate Z ---
        self.layout_edit.addWidget(QLabel("<u>Rotate Z:</u>"))
        self.spin_edit_z_rot = self._create_spinbox(-360, 360, " \u00b0", 0.0, lambda: None)
        self.layout_edit.addWidget(self.spin_edit_z_rot)
        hz = QHBoxLayout()
        for label, delta in [("-5", -5), ("-1", -1), ("+1", 1), ("+5", 5)]:
            b = QPushButton(label)
            b.setFixedWidth(50)
            b.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_z_rot, d))
            hz.addWidget(b)
        self.layout_edit.addLayout(hz)
        self.layout_edit.addSpacing(10)

        self.btn_apply_edit = QPushButton("Apply Transform")
        self.btn_apply_edit.clicked.connect(self._apply_edit_transform)
        self.layout_edit.addWidget(self.btn_apply_edit)

        self.btn_reset_edit = QPushButton("Reset to Original")
        self.btn_reset_edit.clicked.connect(self._reset_edit_transform)
        self.layout_edit.addWidget(self.btn_reset_edit)

        self.layout_edit.addSpacing(15)

        self.btn_save_edit = QPushButton("Save to CSV")
        self.btn_save_edit.clicked.connect(self._save_edit_to_csv)
        self.layout_edit.addWidget(self.btn_save_edit)

        self.lbl_edit_status = QLabel("")
        self.lbl_edit_status.setWordWrap(True)
        self.layout_edit.addWidget(self.lbl_edit_status)

        self.layout_edit.addStretch()

    def _edit_increment(self, spinbox, delta):
        """Increment a spinbox by delta and auto-apply the transform."""
        spinbox.setValue(spinbox.value() + delta)
        self._apply_edit_transform()

    def _apply_edit_transform(self):
        """Recompute points from original data + current X/Y/Z-rot offsets."""
        if self._original_points_xyz is None:
            self.lbl_edit_status.setText("No data loaded.")
            return

        dx = self.spin_edit_x.value()
        dy = self.spin_edit_y.value()
        angle_deg = self.spin_edit_z_rot.value()

        pts, oris = apply_planar_edit_transform(
            self._original_points_xyz,
            self._original_orientations_abc,
            dx,
            dy,
            angle_deg
        )

        self.points_xyz = pts
        if oris is not None:
            self.orientations_abc = oris

        self.lbl_edit_status.setText(
            f"X {dx:+.1f} mm | Y {dy:+.1f} mm | Z rot {angle_deg:+.1f}\u00b0")

        self.update_plot()
        self.plotter.reset_camera()

    def _reset_edit_transform(self):
        """Reset transform to zero and restore original data."""
        self.spin_edit_x.setValue(0.0)
        self.spin_edit_y.setValue(0.0)
        self.spin_edit_z_rot.setValue(0.0)
        self._apply_edit_transform()
        self.lbl_edit_status.setText("Reset to original.")

    def _save_edit_to_csv(self):
        """Save transformed coordinates back to the CSV file."""
        if self.points_xyz is None or not self.last_file_path:
            self.lbl_edit_status.setText("No data or file loaded.")
            return

        if not os.path.exists(self.last_file_path):
            self.lbl_edit_status.setText("Source CSV file not found.")
            return

        try:
            rewrite_trajectory_csv(self.last_file_path, self.points_xyz, self.orientations_abc)

            # Transformed data is now the new baseline
            self._original_points_xyz = self.points_xyz.copy()
            if self.orientations_abc is not None:
                self._original_orientations_abc = self.orientations_abc.copy()
            self.spin_edit_x.setValue(0.0)
            self.spin_edit_y.setValue(0.0)
            self.spin_edit_z_rot.setValue(0.0)

            self.lbl_edit_status.setText(
                f"Saved to {os.path.basename(self.last_file_path)}. Transform reset.")

        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save CSV:\n{e}")
            self.lbl_edit_status.setText("Save failed.")

    def _create_export_tab(self):
        """Build the 'Export' tab: postprocessor selection and actions."""
        self.tab_export = QWidget()
        self.layout_export = QVBoxLayout(self.tab_export)
        self.tabs.addTab(self.tab_export, "Export")

        self.layout_export.addWidget(QLabel("<b>EXPORT & POSTPROCESS</b>"))

        self.combo_postprocessor = QComboBox()
        self.layout_export.addWidget(self.combo_postprocessor)

        self.btn_preview = QPushButton("Preview Script")
        self.btn_preview.clicked.connect(self.on_preview_clicked)
        self.layout_export.addWidget(self.btn_preview)

        self.btn_export = QPushButton("Export Generated Code")
        self.btn_export.clicked.connect(self.on_export_clicked)
        self.layout_export.addWidget(self.btn_export)

        self.populate_postprocessors()
        self.layout_export.addStretch()

    def _create_viewport(self):
        """Build the right-hand 3D viewport, sliders, and color strip."""
        self.right_panel = QWidget()
        self.right_layout = QVBoxLayout(self.right_panel)
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.addWidget(self.right_panel, stretch=1)

        # Plot area (plotter + vertical layer slider)
        self.plot_h_layout = QHBoxLayout()
        self.right_layout.addLayout(self.plot_h_layout, stretch=1)

        self.plotter = QtInteractor(self.central_widget)
        self.plotter.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.plot_h_layout.addWidget(self.plotter.interactor, stretch=1)

        # Vertical layer slider
        self.v_frame = QFrame()
        self.v_layout = QVBoxLayout(self.v_frame)
        self.v_layout.setContentsMargins(5, 0, 0, 0)
        self.plot_h_layout.addWidget(self.v_frame)

        self.lbl_v_layer = QLabel("Layer:\n0 / 0")
        self.lbl_v_layer.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.v_layout.addWidget(self.lbl_v_layer)

        self.v_slider = QSlider(Qt.Orientation.Vertical)
        self.v_slider.setInvertedAppearance(False)
        self.v_slider.setEnabled(False)
        self.v_slider.valueChanged.connect(self.on_v_slider)
        self.v_layout.addWidget(self.v_slider, alignment=Qt.AlignmentFlag.AlignHCenter)

        # Bottom horizontal slider + color strip
        self.bot_frame = QFrame()
        self.bot_layout = QVBoxLayout(self.bot_frame)
        self.bot_layout.setContentsMargins(0, 5, 0, 0)
        self.right_layout.addWidget(self.bot_frame)

        self.lbl_h_progress = QLabel("Progress: 0 / 0 points")
        self.bot_layout.addWidget(self.lbl_h_progress)

        self.h_slider = QSlider(Qt.Orientation.Horizontal)
        self.h_slider.setEnabled(False)
        self.h_slider.valueChanged.connect(self.on_h_slider)
        self.bot_layout.addWidget(self.h_slider)
        self.h_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.h_slider.setTickInterval(1000)

        self.color_strip = ColorStripWidget()
        self.bot_layout.addWidget(self.color_strip)

    # ------------------------------------------------------------------
    # Scene setup
    # ------------------------------------------------------------------

    def setup_scene(self):
        """Initialize the 3D scene: background, axes, table, and load saved settings."""
        self.plotter.set_background('#f0f0f0')
        self.plotter.remove_bounding_box()

        self.draw_table()

        # 3D axes at origin (50 mm)
        origin = (0.0, 0.0, 0.0)
        for direction, color in [((1, 0, 0), 'red'), ((0, 1, 0), 'green'), ((0, 0, 1), 'blue')]:
            arrow = pv.Arrow(start=origin, direction=direction, scale=50.0, shaft_radius=0.03, tip_radius=0.08)
            self.plotter.add_mesh(arrow, color=color, show_edges=False)

        self.view_isometric()

        # Load persisted settings
        self.load_settings()

        # Auto-load last files
        if self.last_file_path and os.path.exists(self.last_file_path):
            self.load_file(self.last_file_path)

        if self.last_urdf_path and os.path.exists(self.last_urdf_path):
            self.load_urdf(self.last_urdf_path)

    def _sync_ui_from_state(self):
        """Push all internal state values into UI widgets without triggering callbacks."""
        self.updating_sliders = True
        try:
            self.lbl_project_name.setText(f"\U0001F4C1 {self._project_name}")
            self.slider_thick.setValue(int(self.print_thickness * 10))
            self.lbl_thickness.setText(f"Extrusion Width:\n{self.print_thickness:.1f} mm")

            self.spin_base_x.setValue(self.base_x)
            self.spin_base_y.setValue(self.base_y)
            self.spin_base_z.setValue(self.base_z)
            self.spin_base_a.setValue(self.base_a)
            self.spin_base_b.setValue(self.base_b)
            self.spin_base_c.setValue(self.base_c)

            self.spin_tool_x.setValue(self.tool_x)
            self.spin_tool_y.setValue(self.tool_y)
            self.spin_tool_z.setValue(self.tool_z)
            self.spin_tool_a.setValue(self.tool_a)
            self.spin_tool_b.setValue(self.tool_b)
            self.spin_tool_c.setValue(self.tool_c)

            self.check_show_table.setChecked(self.show_table)
            self.check_show_robot.setChecked(self.show_robot)

            self.spin_table_x1.setValue(self.table_x1)
            self.spin_table_y1.setValue(self.table_y1)
            self.spin_table_x2.setValue(self.table_x2)
            self.spin_table_y2.setValue(self.table_y2)

            ik_idx = self.combo_ik_config.findText(self.ik_config)
            if ik_idx >= 0:
                self.combo_ik_config.setCurrentIndex(ik_idx)

            self._select_plugin_by_file_name(self.combo_postprocessor, self.last_postprocessor)
        finally:
            self.updating_sliders = False

    # ------------------------------------------------------------------
    # Table & Robot visibility
    # ------------------------------------------------------------------

    def draw_table(self):
        """Draw or remove the build-plate rectangle in the 3D scene."""
        if self.table_actor:
            self.plotter.remove_actor(self.table_actor)
            self.table_actor = None

        if self.show_table:
            width = abs(self.table_x2 - self.table_x1)
            height = abs(self.table_y2 - self.table_y1)
            if width > 0 and height > 0:
                center_x = (self.table_x1 + self.table_x2) / 2.0
                center_y = (self.table_y1 + self.table_y2) / 2.0
                plane = pv.Plane(center=(center_x, center_y, -0.5), direction=(0, 0, 1), i_size=width, j_size=height)
                self.table_actor = self.plotter.add_mesh(plane, color='#303030', opacity=0.8, show_edges=True, edge_color='black', reset_camera=False)

        self.plotter.render()

    def on_table_changed(self):
        """Handle table visibility or dimension changes."""
        if self.updating_sliders:
            return
        self.show_table = self.check_show_table.isChecked()
        self.table_x1 = self.spin_table_x1.value()
        self.table_y1 = self.spin_table_y1.value()
        self.table_x2 = self.spin_table_x2.value()
        self.table_y2 = self.spin_table_y2.value()
        self.draw_table()
        self.save_settings()

    def on_robot_changed(self):
        """Toggle robot mesh visibility."""
        if self.updating_sliders:
            return
        self.show_robot = self.check_show_robot.isChecked()
        for actor in self.robot_actors.values():
            actor.SetVisibility(self.show_robot)
        self.plotter.render()
        self.save_settings()

    def on_transform_changed(self):
        """Handle base/tool transform spinbox changes."""
        if self.updating_sliders:
            return
        self.base_x = self.spin_base_x.value()
        self.base_y = self.spin_base_y.value()
        self.base_z = self.spin_base_z.value()
        self.base_a = self.spin_base_a.value()
        self.base_b = self.spin_base_b.value()
        self.base_c = self.spin_base_c.value()

        self.tool_x = self.spin_tool_x.value()
        self.tool_y = self.spin_tool_y.value()
        self.tool_z = self.spin_tool_z.value()
        self.tool_a = self.spin_tool_a.value()
        self.tool_b = self.spin_tool_b.value()
        self.tool_c = self.spin_tool_c.value()

        self.save_settings()

        if self.points_xyz is not None:
            self._request_update()

    def on_ik_config_changed(self, text):
        """Handle change in IK Initial Configuration."""
        if getattr(self, "updating_sliders", False):
            return
        self.ik_config = text
        self._force_ik_seed_from_config = True
        self.save_settings()
        if self.points_xyz is not None:
            self._request_update()

    # ------------------------------------------------------------------
    # Postprocessor
    # ------------------------------------------------------------------

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
        self._postprocess_ticket = ticket

        def on_finished(exit_code, exit_status):
            if not self.task_controller.is_current(ticket):
                return
            self._end_job(ticket)
            if exit_code == 0 and self._file_was_updated(output_file, previous_state):
                self.lbl_status.setText(success_text)
                if success_callback is not None:
                    success_callback(output_file)
                return

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
        self._qprocess.setWorkingDirectory(os.path.dirname(os.path.abspath(__file__)))
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

        app_dir = os.path.dirname(os.path.abspath(__file__))
        out_dir = os.path.join(app_dir, "Output")
        os.makedirs(out_dir, exist_ok=True)
        preview_file = os.path.join(out_dir, "preview.txt")
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

    # ------------------------------------------------------------------
    # Camera shortcuts
    # ------------------------------------------------------------------

    def view_isometric(self):
        """Reset camera to isometric view."""
        self.plotter.view_isometric()
        self.plotter.camera.zoom(1.2)

    def view_top(self):
        """Set camera to top-down (XY) view."""
        self.plotter.view_xy()
        self.plotter.camera.roll = 0

    def view_side(self):
        """Set camera to side (XZ) view."""
        self.plotter.view_xz()
        self.plotter.camera.roll = 0

    # ------------------------------------------------------------------
    # File loading
    # ------------------------------------------------------------------

    def populate_importers(self):
        """Scan Importer/ dir, dynamically load each plugin, fill combobox."""
        self.combo_importer.clear()
        self._importer_specs = []

        self._importer_specs = self._discover_plugins("Importer", required_attrs=("run_import",))
        for spec in self._importer_specs:
            self.combo_importer.addItem(spec.display_name, userData=spec)

        self.btn_run_import.setEnabled(
            bool(self._importer_specs) and bool(self._import_gcode_path))

    def _select_gcode_for_import(self):
        """Open GCODE/ dialog, call each plugin's can_handle() to pre-select."""
        app_dir = os.path.dirname(os.path.abspath(__file__))
        gcode_dir = os.path.join(app_dir, "GCODE")
        os.makedirs(gcode_dir, exist_ok=True)

        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select G-Code File", gcode_dir,
            "G-Code Files (*.gcode *.gco *.nc);;All Files (*)")
        if not file_path:
            return

        self._import_gcode_path = file_path
        self.lbl_import_file.setText(os.path.basename(file_path))

        # Ask each plugin whether it can handle this file
        matched = -1
        for i, spec in enumerate(self._importer_specs):
            try:
                if hasattr(spec.module, 'can_handle') and spec.module.can_handle(file_path):
                    matched = i
                    break
            except Exception:
                continue

        if matched >= 0:
            self.combo_importer.setCurrentIndex(matched)
            self.lbl_import_status.setText(
                f"Auto-detected: '{self.combo_importer.itemText(matched)}' selected. "
                f"Override if needed, then click Import & Load.")
        else:
            self.lbl_import_status.setText(
                "Auto-detection found no match. Select an importer manually.")

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

        app_dir = os.path.dirname(os.path.abspath(__file__))
        csv_dir = os.path.join(app_dir, "CSV")
        os.makedirs(csv_dir, exist_ok=True)
        name_only, _ = os.path.splitext(os.path.basename(self._import_gcode_path))
        csv_out_path = os.path.join(csv_dir, name_only + ".csv")

        self.lbl_import_status.setText(f"Running {fname}...")
        ticket = self._begin_job("import", f"Running {fname}...")
        self._import_ticket = ticket

        self._import_thread = ImporterThread(mod, self._import_gcode_path, csv_out_path)
        self._import_thread.finished_signal.connect(lambda success, payload, tk=ticket: self._on_import_finished(tk, success, payload))
        self._import_thread.start()

    def _on_import_finished(self, ticket, success: bool, payload: str):
        """Handle ImporterThread completion."""
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        if success:
            self.lbl_import_status.setText("Import done. Loading CSV...")
            self.lbl_status.setText("Import successful.")
            self.load_file(payload)  # payload = output CSV path
        else:
            QMessageBox.critical(self, "Import Error",
                                 f"Importer raised an error:\n\n{payload}")
            self.lbl_import_status.setText("Import failed.")
            self.lbl_status.setText("Import failed.")

    def load_file_dialog(self):
        """Open a file dialog to select and load a CSV file."""
        app_dir = os.path.dirname(os.path.abspath(__file__))
        csv_dir = os.path.join(app_dir, "CSV")
        os.makedirs(csv_dir, exist_ok=True)
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select CSV Data Stream", csv_dir,
            "CSV Files (*.csv);;All Files (*)")
        if not file_path:
            return
        self.load_file(file_path)

    def load_urdf_dialog(self):
        """Scan for URDF files and show a selection dialog."""
        from PyQt6.QtWidgets import QInputDialog

        app_dir = os.path.dirname(os.path.abspath(__file__))
        self._robot_descriptors = discover_robot_descriptors(app_dir)
        if not self._robot_descriptors:
            QMessageBox.information(self, "No Robots Found",
                                   "No URDF files found in subdirectories of ROOT.")
            return

        labels = [descriptor.label for descriptor in self._robot_descriptors]
        current = 0
        if self.last_urdf_path:
            for i, descriptor in enumerate(self._robot_descriptors):
                if os.path.normpath(descriptor.file_path) == os.path.normpath(self.last_urdf_path):
                    current = i
                    break

        chosen, ok = QInputDialog.getItem(
            self, "Select Robot", "Available robots:",
            labels, current, False)
        if not ok:
            return

        idx = labels.index(chosen)
        self.load_urdf(self._robot_descriptors[idx].file_path)

    def load_urdf(self, file_path):
        """Load a URDF robot model and add its meshes to the scene."""
        if not os.path.exists(file_path):
            self.lbl_status.setText("URDF file not found.")
            return

        ticket = self._begin_job("robot_load", f"Loading Robot: {os.path.basename(file_path)}...")
        self._robot_ticket = ticket
        self._robot_thread = RobotLoadThread(RobotSimulator, file_path)
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
            actor = self.plotter.add_mesh(mesh, color="#c0c0c0", show_edges=True)
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
            self.lbl_status.setText("Last saved file not found.")
            return

        self.last_file_path = file_path
        ticket = self._begin_job("load", f"Loading {os.path.basename(file_path)}...")
        self._load_ticket = ticket

        self.loader_thread = DataLoaderThread(file_path)
        self.loader_thread.finished_signal.connect(
            lambda points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer, estimated_time_s, estimated_weight_g, tk=ticket:
                self.on_load_finished(tk, points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer, estimated_time_s, estimated_weight_g)
        )
        self.loader_thread.error_signal.connect(lambda err_msg, tk=ticket: self.on_load_error(tk, err_msg))
        self.loader_thread.start()

    def on_load_error(self, ticket, err_msg):
        """Handle CSV loading errors."""
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        QMessageBox.critical(self, "Error Loading File", err_msg)
        self.lbl_status.setText("Load failed.")

    def on_load_finished(self, ticket, points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer, estimated_time_s, estimated_weight_g):
        """Handle successful CSV load: store data and update UI."""
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        if points_xyz is None or len(points_xyz) == 0:
            QMessageBox.warning(self, "No Data", "No valid points found in CSV.")
            self.lbl_status.setText("Load failed.")
            return

        self.points_xyz = points_xyz
        self._original_points_xyz = points_xyz.copy()
        self._original_orientations_abc = orientations_abc.copy()

        # Reset edit transform when new data is loaded
        self.spin_edit_x.setValue(0.0)
        self.spin_edit_y.setValue(0.0)
        self.spin_edit_z_rot.setValue(0.0)
        self.lbl_edit_status.setText("")
        self.orientations_abc = orientations_abc
        self.colors_rgb = colors_rgb
        self.layer_end_indices = layer_ends
        self.max_layer = max_layer
        self.trajectory_renderer.set_trajectory(self.points_xyz, self.colors_rgb, self.print_thickness)

        # Pre-sort layer ends by end_point for fast binary search in on_h_slider
        self._sorted_layer_ends = sorted(layer_ends.items(), key=lambda kv: kv[1])

        num_pts = len(self.points_xyz)

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

        self.current_step = num_pts
        self.max_layer = max_layer
        self.estimated_time_s = estimated_time_s
        self.estimated_weight_g = estimated_weight_g
        self._reset_ik_tracking()

        self.current_layer = max_layer

        self.update_ui_labels()
        self.update_plot()

        self.plotter.reset_camera()

        # Display estimated print time
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

    # ------------------------------------------------------------------
    # Slider event handlers (with debounce)
    # ------------------------------------------------------------------

    def on_thickness_changed(self, val):
        """Handle extrusion width slider change."""
        if getattr(self, "updating_sliders", False):
            return
        self.print_thickness = float(val) / 10.0
        self.lbl_thickness.setText(f"Extrusion Width:\n{self.print_thickness:.1f} mm")
        self.save_settings()
        if self.points_xyz is not None:
            self.trajectory_renderer.set_trajectory(self.points_xyz, self.colors_rgb, self.print_thickness)
            self._request_update()

    def on_v_slider(self, val):
        """Handle vertical (layer) slider change."""
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
        """Handle horizontal (point progress) slider change."""
        if self.updating_sliders or self.points_xyz is None:
            return

        self.current_step = val

        # Binary search: find the layer whose end_point is >= val
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
        """Refresh the text labels showing current progress."""
        num_pts = len(self.points_xyz) if self.points_xyz is not None else 0
        self.lbl_h_progress.setText(f"Progress: {self.current_step} / {num_pts} points")
        self.lbl_v_layer.setText(f"Layer:\n{self.current_layer} / {self.max_layer}")

    # ------------------------------------------------------------------
    # Rendering (with debounce for slider performance)
    # ------------------------------------------------------------------

    def _request_update(self):
        """Request a debounced plot update â€” restarts the timer on each call."""
        self._pending_tube_render = True
        self._render_timer.start()

    def _deferred_update_plot(self):
        """Called by debounce timer â€” performs the actual rendering."""
        if self._pending_tube_render:
            self._pending_tube_render = False
            self.update_plot()

    def update_plot(self):
        """Render the cached trajectory subset up to current_step."""
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

    def _get_ik_seed(self, target_pos=None):
        """Build initial position array from selected IK_CONFIGS, with dynamic base orientation."""
        if not self.robot_sim.urdf_model:
            return None
            
        angles = IK_CONFIGS.get(self.ik_config, IK_CONFIGS["Default (Zero)"])
        seed = self.robot_sim.build_seed_from_active_angles(angles)
        if target_pos is not None and seed is not None:
            seed = self.robot_sim.apply_base_azimuth_to_seed(seed, target_pos)
        return seed

    def _get_ik_seed_candidates(self):
        """Return IK seed candidates (selected config first, then alternatives)."""
        if not self.robot_sim.urdf_model:
            return []

        candidates = []
        seen = set()
        ordered_names = [self.ik_config] + [k for k in IK_CONFIGS.keys() if k != self.ik_config]

        for cfg_name in ordered_names:
            angles = IK_CONFIGS.get(cfg_name, IK_CONFIGS["Default (Zero)"])
            seed = self.robot_sim.build_seed_from_active_angles(angles)
            if seed is None:
                continue
            key = tuple(np.round(seed, 6))
            if key not in seen:
                seen.add(key)
                candidates.append(seed)

        return candidates

    def _reset_ik_tracking(self, force_config_seed=False):
        """Drop cached IK continuity state after a real context change."""
        self.last_ik_solution = None
        self._force_ik_seed_from_config = force_config_seed

    def _update_robot_ik(self, target_pt, target_ori):
        """Compute IK for the robot to follow the target point and update visuals."""
        if not self.robot_sim.urdf_model or not self.show_robot:
            return

        try:
            t_point = kuka_base_to_matrix(
                target_pt[0], target_pt[1], target_pt[2],
                target_ori[0], target_ori[1], target_ori[2]
            )

            t_base = kuka_base_to_matrix(self.base_x, self.base_y, self.base_z, self.base_a, self.base_b, self.base_c)
            t_base_inv = np.linalg.inv(t_base)

            t_tool = kuka_base_to_matrix(self.tool_x, self.tool_y, self.tool_z, self.tool_a, self.tool_b, self.tool_c)
            t_tool_inv = np.linalg.inv(t_tool)

            t_tcp_target = t_base @ t_point
            t_flange_target = t_tcp_target @ t_tool_inv

            target_pos = t_flange_target[0:3, 3] / 1000.0
            target_ori = t_flange_target[0:3, 0:3]

            if self.last_ik_solution is not None and not self._force_ik_seed_from_config:
                seed_position = self.last_ik_solution
            else:
                seed_position = self._get_ik_seed(target_pos=target_pos)
                
            ik_solution = self.robot_sim.calculate_ik(
                target_pos,
                target_orientation=target_ori,
                initial_position=seed_position
            )
            if ik_solution is not None:
                self.last_ik_solution = np.copy(ik_solution)
                self._force_ik_seed_from_config = False
                
                # Update visual labels
                act_j = []
                for idx, link in enumerate(self.robot_sim.ik_chain.links):
                    if link.name in self.robot_sim.active_joints and len(act_j) < 6:
                        act_j.append(math.degrees(ik_solution[idx]))
                if len(act_j) == 6:
                    text_j123 = f"J1-3 deg: A1:{act_j[0]:.1f} A2:{act_j[1]:.1f} A3:{act_j[2]:.1f}"
                    text_j456 = f"J4-6 deg: A4:{act_j[3]:.1f} A5:{act_j[4]:.1f} A6:{act_j[5]:.1f}"
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
                limit_violations = evaluation.limit_violations
                singularities = evaluation.singularities

                transforms = self.robot_sim.get_forward_transforms(ik_solution)
                for link_name, t_matrix in transforms.items():
                    if link_name in self.robot_actors:
                        actor = self.robot_actors[link_name]
                        actor.user_matrix = t_base_inv @ t_matrix

                        if link_name in singularities:
                            actor.prop.color = 'yellow'
                        elif link_name in limit_violations:
                            actor.prop.color = 'red'
                        else:
                            actor.prop.color = '#c0c0c0'
        except Exception as e:
            print(f"IK error: {e}")

    # ------------------------------------------------------------------
    # Trajectory testing
    # ------------------------------------------------------------------

    def run_trajectory_test(self):
        """Start background trajectory feasibility test for all points."""
        if self.points_xyz is None or not self.robot_sim.urdf_model:
            return

        ticket = self._begin_job("trajectory_test", "Testing trajectory... This might take a while.")
        self._traj_ticket = ticket

        base_params = (self.base_x, self.base_y, self.base_z, self.base_a, self.base_b, self.base_c)
        tool_params = (self.tool_x, self.tool_y, self.tool_z, self.tool_a, self.tool_b, self.tool_c)
        seed_candidates = self._get_ik_seed_candidates()
        if not seed_candidates:
            fallback_seed = self._get_ik_seed()
            if fallback_seed is not None:
                seed_candidates = [fallback_seed]

        self.traj_thread = TrajectoryTestThread(
            self.points_xyz, self.orientations_abc, self.robot_sim,
            base_params, tool_params, seed_candidates
        )
        self.traj_thread.progress_signal.connect(lambda current, total, tk=ticket: self.on_traj_test_progress(tk, current, total))
        self.traj_thread.finished_signal.connect(lambda statuses, tk=ticket: self.on_traj_test_finished(tk, statuses))
        self.traj_thread.start()

    def on_traj_test_progress(self, ticket, current, total):
        """Update status label with trajectory test progress."""
        if not self.task_controller.is_current(ticket):
            return
        self.lbl_status.setText(f"Testing trajectory: {current} / {total} points checked")

    def on_traj_test_finished(self, ticket, statuses):
        """Handle trajectory test completion."""
        if not self.task_controller.is_current(ticket):
            return
        self._end_job(ticket)
        self.lbl_status.setText("Test complete.")

        if statuses is not None and len(statuses) == len(self.points_xyz):
            self.color_strip.set_statuses(statuses)

        self._restore_interaction_state()

    # ------------------------------------------------------------------
    # Settings persistence
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Project management
    # ------------------------------------------------------------------

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
            ik_config=self.ik_config or DEFAULT_IK_CONFIG
        )

    def _load_meta(self):
        """Read viewer_settings.json to determine which project to open."""
        self._project_file = self._project_store.load_last_project_path()
        self._project_name = self._project_store.project_name_from_path(self._project_file)

    def _save_meta(self):
        """Write the current project name to viewer_settings.json."""
        try:
            self._project_store.save_last_project_path(self._project_file)
        except Exception as e:
            print(f"Error saving meta: {e}")

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
        except Exception as e:
            print(f"Error saving project: {e}")

    def _new_project(self):
        """Create a new project with default settings."""
        from PyQt6.QtWidgets import QInputDialog
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

        # Save current project first
        self._save_project()

        # Reset to defaults
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
        from PyQt6.QtWidgets import QInputDialog
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
            self, "Open Project", "Available projects:",
            labels, current, False)
        if not ok:
            return

        # Save current project before switching
        self._save_project()

        idx = labels.index(chosen)
        path = projects[idx]
        self._load_project(path)
        self._save_meta()

        # Reload robot if changed
        if self.last_urdf_path and os.path.exists(self.last_urdf_path):
            self.load_urdf(self.last_urdf_path)

        # Reload CSV/NC file if present
        if self.last_file_path and os.path.exists(self.last_file_path):
            self.load_file(self.last_file_path)

        self.lbl_status.setText(f"Opened project '{self._project_name}'.")

    def _save_project_as(self):
        """Save the current project under a new name."""
        from PyQt6.QtWidgets import QInputDialog
        name, ok = QInputDialog.getText(
            self, "Save Project As", "New project name:",
            text=self._project_name)
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


if __name__ == "__main__":
    multiprocessing.freeze_support()
    app = QApplication(sys.argv)

    # ---------------------------------------------------------------------------
    # Splash screen â€” shown IMMEDIATELY, before any heavy imports.
    #
    # pyvista (VTK) and robot_ik (ikpy/yourdfpy/trimesh) are slow to initialise
    # (~3-6 s combined). Deferring them here means the splash appears at once,
    # giving the user instant feedback that the app is loading.
    # ---------------------------------------------------------------------------
    _app_dir = os.path.dirname(os.path.abspath(__file__))
    _splash_path = os.path.join(_app_dir, "splash.png")
    splash = None
    if os.path.exists(_splash_path):
        from PyQt6.QtGui import QPixmap
        _pixmap = QPixmap(_splash_path)
        _screen = app.primaryScreen().geometry()
        _max_w = int(_screen.width() * 0.8)
        _max_h = int(_screen.height() * 0.8)
        if _pixmap.width() > _max_w or _pixmap.height() > _max_h:
            _pixmap = _pixmap.scaled(_max_w, _max_h,
                                     Qt.AspectRatioMode.KeepAspectRatio,
                                     Qt.TransformationMode.SmoothTransformation)
        # WindowStaysOnTopHint: splash stays above everything, including the
        # main window as it initialises underneath.
        splash = QSplashScreen(
            _pixmap,
            Qt.WindowType.WindowStaysOnTopHint | Qt.WindowType.SplashScreen
        )
        splash.show()
        splash.raise_()
        app.processEvents()

    # ---------------------------------------------------------------------------
    # Heavy imports â€” happen here while the splash is visible.
    # Assigning to module-level globals makes them available to all class methods.
    # ---------------------------------------------------------------------------
    import pyvista as _pv                           # noqa: E402
    from pyvistaqt import QtInteractor as _QI      # noqa: E402
    from robot_ik import RobotSimulator as _RS     # noqa: E402

    # Patch the module-level placeholders set at the top of this file.
    import sys as _sys
    _mod = _sys.modules[__name__]
    _mod.pv = _pv
    _mod.QtInteractor = _QI
    _mod.RobotSimulator = _RS

    app.processEvents()  # keep UI alive while imports settle

    # ---------------------------------------------------------------------------
    # Build main window and start minimum-display countdown.
    # ---------------------------------------------------------------------------
    from PyQt6.QtCore import QElapsedTimer
    _load_timer = QElapsedTimer()
    _load_timer.start()

    window = RobotPathViewer()
    window.show()

    # Close splash only after at least 4 s from end-of-imports AND after the
    # main window has been shown (splash.finish waits for the first paint).
    if splash is not None:
        _MIN_MS = 3000
        _remaining = max(0, _MIN_MS - _load_timer.elapsed())
        QTimer.singleShot(_remaining, lambda: splash.finish(window))

    sys.exit(app.exec())

