import os
import json
import subprocess
import sys
import multiprocessing
from concurrent.futures import ProcessPoolExecutor, as_completed
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

# robot_ik (ikpy / yourdfpy / trimesh) is also imported lazily in __main__.
RobotSimulator = None


# ---------------------------------------------------------------------------
# Shared utility: KUKA ZYX Euler → 4×4 homogeneous matrix
# ---------------------------------------------------------------------------
def kuka_base_to_matrix(x, y, z, a, b, c):
    """Convert KUKA BASE/TOOL parameters (X,Y,Z,A,B,C) to a 4×4 homogeneous
    transformation matrix using ZYX (A-B-C) Euler convention."""
    an = np.radians(a)
    bn = np.radians(b)
    cn = np.radians(c)
    cz, sz = np.cos(an), np.sin(an)
    cy, sy = np.cos(bn), np.sin(bn)
    cx, sx = np.cos(cn), np.sin(cn)
    Rz = np.array([[cz, -sz, 0, 0], [sz, cz, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    Ry = np.array([[cy, 0, sy, 0], [0, 1, 0, 0], [-sy, 0, cy, 0], [0, 0, 0, 1]])
    Rx = np.array([[1, 0, 0, 0], [0, cx, -sx, 0], [0, sx, cx, 0], [0, 0, 0, 1]])
    R = Rz @ Ry @ Rx
    R[0, 3] = x
    R[1, 3] = y
    R[2, 3] = z
    return R


# ---------------------------------------------------------------------------
# Segment color constants (RGB 0-255)
# ---------------------------------------------------------------------------
COLOR_PRINT   = np.array([44, 160, 44],   dtype=np.uint8)   # Green
COLOR_RETRACT = np.array([214, 39, 40],   dtype=np.uint8)   # Red
COLOR_TRAVEL  = np.array([127, 127, 127], dtype=np.uint8)   # Grey


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


# ---------------------------------------------------------------------------
# Multiprocessing worker for trajectory testing
# ---------------------------------------------------------------------------
def _trajectory_worker(urdf_path, points_chunk, base_params, tool_params):
    """Process a chunk of trajectory points in a separate process.

    Each chunk runs sequentially with IK seeding: the solution for point N
    is used as the initial guess for point N+1. This dramatically reduces
    the number of scipy optimizer iterations since adjacent points are close.

    Returns:
        numpy array of int8 statuses for this chunk.
    """
    from robot_ik import RobotSimulator

    sim = RobotSimulator()
    sim.load_robot(urdf_path)

    b_x, b_y, b_z, b_a, b_b, b_c = base_params
    t_x, t_y, t_z, t_a, t_b, t_c = tool_params

    t_base = kuka_base_to_matrix(b_x, b_y, b_z, b_a, b_b, b_c)
    t_tool = kuka_base_to_matrix(t_x, t_y, t_z, t_a, t_b, t_c)
    t_tool_inv = np.linalg.inv(t_tool)

    n = len(points_chunk)
    statuses = np.zeros(n, dtype=np.int8)
    prev_solution = None

    for i, pt in enumerate(points_chunk):
        t_point = np.eye(4)
        t_point[0:3, 3] = pt
        t_flange_target = (t_base @ t_point) @ t_tool_inv

        target_pos = t_flange_target[0:3, 3] / 1000.0
        target_ori = t_flange_target[0:3, 0:3]

        ik_solution = sim.calculate_ik(
            target_pos,
            target_orientation=target_ori,
            initial_position=prev_solution
        )

        if ik_solution is None:
            statuses[i] = 3  # Unreachable
        else:
            prev_solution = ik_solution  # seed next iteration
            if len(sim.check_limits(ik_solution)) > 0:
                statuses[i] = 2  # Limit
            elif len(sim.check_singularities(ik_solution)) > 0:
                statuses[i] = 1  # Singularity
            else:
                statuses[i] = 0  # OK

    return statuses


class TrajectoryTestThread(QThread):
    """Background thread that splits trajectory across CPU cores for parallel IK testing.

    Optimizations over the naive approach:
    1. IK seeding — each point uses the previous solution as the optimizer start,
       reducing convergence time by 5–10×.
    2. Multiprocessing — trajectory is divided into chunks processed in parallel,
       giving an additional N× speedup where N = number of CPU cores.
    """

    progress_signal = pyqtSignal(int, int)   # current, total
    finished_signal = pyqtSignal(object)     # numpy array of statuses

    def __init__(self, points_xyz, robot_sim, base_params, tool_params):
        super().__init__()
        self.points_xyz = points_xyz
        self.urdf_path = robot_sim.urdf_path
        self.has_model = robot_sim.urdf_model is not None
        self.base_params = base_params
        self.tool_params = tool_params

    def run(self):
        if not self.has_model:
            self.finished_signal.emit(None)
            return

        total = len(self.points_xyz)
        num_workers = max(1, min(multiprocessing.cpu_count(), 8))
        chunk_size = max(100, total // num_workers)

        # Split the points array into chunks for parallel processing
        chunks = []
        for start in range(0, total, chunk_size):
            end = min(start + chunk_size, total)
            chunks.append((start, self.points_xyz[start:end]))

        self.progress_signal.emit(0, total)

        statuses = np.zeros(total, dtype=np.int8)
        completed = 0

        try:
            with ProcessPoolExecutor(max_workers=num_workers) as pool:
                futures = {}
                for start, chunk_pts in chunks:
                    future = pool.submit(
                        _trajectory_worker,
                        self.urdf_path,
                        chunk_pts,
                        self.base_params,
                        self.tool_params
                    )
                    futures[future] = (start, len(chunk_pts))

                for future in as_completed(futures):
                    start, length = futures[future]
                    chunk_result = future.result()
                    statuses[start:start + length] = chunk_result
                    completed += length
                    self.progress_signal.emit(completed, total)

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
        """Parse CSV and build numpy arrays directly (fast path).

        Expected CSV format (14 columns):
        TYPE;X;Y;Z;A;B;C;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS
        """
        try:
            types = []
            xs, ys, zs = [], [], []
            a_angles, b_angles, c_angles = [], [], []
            e_ratios = []
            tcp_speeds = []
            layer_indices = []
            skipped = 0

            with open(self.file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue

                    parts = line.split(';')
                    if len(parts) >= 12:
                        try:
                            types.append(parts[0])
                            xs.append(float(parts[1]))
                            ys.append(float(parts[2]))
                            zs.append(float(parts[3]))
                            a_angles.append(float(parts[4]))
                            b_angles.append(float(parts[5]))
                            c_angles.append(float(parts[6]))
                            e_ratios.append(float(parts[8]))
                            tcp_speeds.append(float(parts[7]))
                            layer_indices.append(int(parts[11]))
                        except ValueError:
                            skipped += 1
                    elif len(parts) >= 9:
                        # Backwards compatibility: old 11-column format without A, B, C
                        try:
                            types.append(parts[0])
                            xs.append(float(parts[1]))
                            ys.append(float(parts[2]))
                            zs.append(float(parts[3]))
                            a_angles.append(0.0)
                            b_angles.append(0.0)
                            c_angles.append(0.0)
                            e_ratios.append(float(parts[5]))
                            tcp_speeds.append(float(parts[4]))
                            layer_indices.append(int(parts[8]))
                        except ValueError:
                            skipped += 1
                    else:
                        skipped += 1

            if not xs:
                self.error_signal.emit(f"No valid data points found in CSV. ({skipped} lines skipped)")
                return

            n = len(xs)
            points_xyz = np.column_stack((xs, ys, zs)).astype(np.float32)
            orientations_abc = np.column_stack((a_angles, b_angles, c_angles)).astype(np.float32)

            # Build colors array for segments (n-1 segments between n points)
            colors_rgb = np.full((n - 1, 3), COLOR_TRAVEL, dtype=np.uint8)
            for i in range(1, n):
                p_type = types[i]
                e_ratio = e_ratios[i]
                if p_type == 'P' and e_ratio > 0:
                    colors_rgb[i - 1] = COLOR_PRINT
                elif p_type in ('R', 'U') or (p_type == 'P' and e_ratio < 0):
                    colors_rgb[i - 1] = COLOR_RETRACT

            # Build layer_ends mapping
            layer_ends = {}
            max_layer = 0
            for idx, li in enumerate(layer_indices):
                layer_ends[li] = idx + 1
                if li > max_layer:
                    max_layer = li

            if skipped > 0:
                print(f"CSV loader: {skipped} lines skipped due to parse errors.")

            # Compute estimated print time and weight
            estimated_time_s = 0.0
            estimated_weight_g = 0.0
            for i in range(1, n):
                dx = xs[i] - xs[i - 1]
                dy = ys[i] - ys[i - 1]
                dz = zs[i] - zs[i - 1]
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                speed = tcp_speeds[i]
                e_ratio = e_ratios[i]  # g/m

                if speed > 0.001:
                    estimated_time_s += dist / speed
                    
                if e_ratio > 0.0:
                    dist_m = dist / 1000.0  # mm to m
                    estimated_weight_g += dist_m * e_ratio

            self.finished_signal.emit(points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer, estimated_time_s, estimated_weight_g)
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

        self.settings_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "viewer_settings.json")

        # Data state
        self.points_xyz = None
        self._original_points_xyz = None
        self.orientations_abc = None
        self.colors_rgb = None
        self.layer_end_indices = {}
        self._sorted_layer_ends = []  # [(layer_idx, end_point), ...] sorted by end_point
        self.max_layer = 0
        self.current_step = 0
        self.current_layer = 0
        self.print_thickness = 1.0
        self.last_file_path = ""
        self.last_postprocessor = ""
        self.last_urdf_path = ""

        # Table state
        self.table_x1 = 0.0
        self.table_y1 = 0.0
        self.table_x2 = 550.0
        self.table_y2 = 650.0
        self.show_table = True
        self.show_robot = True
        self.table_actor = None

        # Transform state
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_z = 0.0
        self.base_a = 0.0
        self.base_b = 0.0
        self.base_c = 0.0

        self.tool_x = 0.0
        self.tool_y = 0.0
        self.tool_z = 0.0
        self.tool_a = 0.0
        self.tool_b = 0.0
        self.tool_c = 0.0

        # Robot simulator
        self.robot_sim = RobotSimulator()
        self.robot_actors = {}

        # Importer plugin modules loaded from Importer/ directory
        self._importer_modules = []  # parallel list to combo_importer items
        self._import_gcode_path = ""  # last selected G-code file

        # Render actors
        self.path_actor_print = None
        self.path_actor_travel = None
        self.tool_actor = None

        self.updating_sliders = False

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

        self.btn_load_urdf = QPushButton("Load URDF Robot...")
        self.btn_load_urdf.clicked.connect(self.load_urdf_dialog)
        self.layout_workplace.addWidget(self.btn_load_urdf)

        self.btn_test_traj = QPushButton("Test Trajectory")
        self.btn_test_traj.clicked.connect(self.run_trajectory_test)
        self.btn_test_traj.setEnabled(False)
        self.layout_workplace.addWidget(self.btn_test_traj)

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
        self.spin_base_a = self._create_spinbox(-360, 360, " °", self.base_a, self.on_transform_changed)
        self.spin_base_b = self._create_spinbox(-360, 360, " °", self.base_b, self.on_transform_changed)
        self.spin_base_c = self._create_spinbox(-360, 360, " °", self.base_c, self.on_transform_changed)
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
        self.spin_tool_a = self._create_spinbox(-360, 360, " °", self.tool_a, self.on_transform_changed)
        self.spin_tool_b = self._create_spinbox(-360, 360, " °", self.tool_b, self.on_transform_changed)
        self.spin_tool_c = self._create_spinbox(-360, 360, " °", self.tool_c, self.on_transform_changed)
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

        # Start from a fresh copy of the original data
        pts = self._original_points_xyz.copy()

        # Rotate around the centroid of the ORIGINAL data
        if angle_deg != 0.0:
            cx = np.mean(pts[:, 0])
            cy = np.mean(pts[:, 1])
            rad = np.radians(angle_deg)
            cos_a, sin_a = np.cos(rad), np.sin(rad)
            rel_x = pts[:, 0] - cx
            rel_y = pts[:, 1] - cy
            pts[:, 0] = cos_a * rel_x - sin_a * rel_y + cx
            pts[:, 1] = sin_a * rel_x + cos_a * rel_y + cy

        # Translate
        pts[:, 0] += dx
        pts[:, 1] += dy

        self.points_xyz = pts

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
            with open(self.last_file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()

            # Rewrite data lines with updated X, Y, Z from points_xyz
            pt_idx = 0
            new_lines = []
            for line in lines:
                stripped = line.strip()
                if not stripped or stripped.startswith('#'):
                    new_lines.append(line)
                    continue

                parts = stripped.split(';')
                if len(parts) >= 4 and pt_idx < len(self.points_xyz):
                    parts[1] = f"{self.points_xyz[pt_idx, 0]:.3f}"
                    parts[2] = f"{self.points_xyz[pt_idx, 1]:.3f}"
                    parts[3] = f"{self.points_xyz[pt_idx, 2]:.3f}"
                    new_lines.append(';'.join(parts) + '\n')
                    pt_idx += 1
                else:
                    new_lines.append(line)

            with open(self.last_file_path, 'w', encoding='utf-8') as f:
                f.writelines(new_lines)

            # Transformed data is now the new baseline
            self._original_points_xyz = self.points_xyz.copy()
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

        # Load persisted settings and sync UI
        self.load_settings()
        self._sync_ui_from_state()

        # Auto-load last files
        if self.last_file_path and os.path.exists(self.last_file_path):
            self.load_file(self.last_file_path)

        if self.last_urdf_path and os.path.exists(self.last_urdf_path):
            self.load_urdf(self.last_urdf_path)

    def _sync_ui_from_state(self):
        """Push all internal state values into UI widgets without triggering callbacks."""
        self.updating_sliders = True

        self.slider_thick.setValue(int(self.print_thickness * 10))

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

    def on_robot_changed(self):
        """Toggle robot mesh visibility."""
        if self.updating_sliders:
            return
        self.show_robot = self.check_show_robot.isChecked()
        for actor in self.robot_actors.values():
            actor.SetVisibility(self.show_robot)
        self.plotter.render()

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

        if self.points_xyz is not None:
            self._request_update()

    # ------------------------------------------------------------------
    # Postprocessor
    # ------------------------------------------------------------------

    def populate_postprocessors(self):
        """Scan the Postprocesor directory and populate the combobox."""
        self.combo_postprocessor.clear()
        post_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Postprocesor")
        if os.path.isdir(post_dir):
            scripts = [f for f in os.listdir(post_dir) if f.endswith('.py')]
            self.combo_postprocessor.addItems(scripts)

    def _run_postprocessor_async(self, script_path, input_file, output_file, on_finished):
        """Run a postprocessor script as a non-blocking QProcess."""
        self._qprocess = QProcess(self)
        self._qprocess.finished.connect(on_finished)
        self._qprocess.start(sys.executable, [script_path, input_file, output_file])

    def on_preview_clicked(self):
        """Run the selected postprocessor and open the preview file."""
        selected = self.combo_postprocessor.currentText()
        if not selected:
            QMessageBox.warning(self, "Warning", "Please select a postprocessor first.")
            return

        if not self.last_file_path or not os.path.exists(self.last_file_path):
            QMessageBox.warning(self, "Warning", "Please load a CSV file first.")
            return

        app_dir = os.path.dirname(os.path.abspath(__file__))
        out_dir = os.path.join(app_dir, "Output")
        os.makedirs(out_dir, exist_ok=True)
        preview_file = os.path.join(out_dir, "preview.txt")
        script_path = os.path.join(app_dir, "Postprocesor", selected)

        self.lbl_status.setText(f"Running preview with {selected}...")

        def on_finished(exit_code, exit_status):
            if exit_code == 0:
                self.lbl_status.setText("Preview generated successfully.")
                if os.name == 'nt':
                    os.startfile(preview_file)
                else:
                    import platform
                    if platform.system() == 'Darwin':
                        subprocess.call(('open', preview_file))
                    else:
                        subprocess.call(('xdg-open', preview_file))
            else:
                QMessageBox.critical(self, "Postprocessor Error", f"Postprocessor exited with code {exit_code}.")
                self.lbl_status.setText("Preview failed.")

        self._run_postprocessor_async(script_path, self.last_file_path, preview_file, on_finished)

    def on_export_clicked(self):
        """Run the selected postprocessor and save to a user-chosen location."""
        selected = self.combo_postprocessor.currentText()
        if not selected:
            QMessageBox.warning(self, "Warning", "Please select a postprocessor first.")
            return

        if not self.last_file_path or not os.path.exists(self.last_file_path):
            QMessageBox.warning(self, "Warning", "Please load a CSV file first.")
            return

        out_file, _ = QFileDialog.getSaveFileName(self, "Export File As", "", "Text Files (*.txt);;All Files (*)")
        if not out_file:
            return

        app_dir = os.path.dirname(os.path.abspath(__file__))
        script_path = os.path.join(app_dir, "Postprocesor", selected)

        self.lbl_status.setText(f"Exporting with {selected}...")

        def on_finished(exit_code, exit_status):
            if exit_code == 0:
                self.lbl_status.setText(f"Exported successfully to {os.path.basename(out_file)}.")
                QMessageBox.information(self, "Success", f"File exported successfully to:\n{out_file}")
            else:
                QMessageBox.critical(self, "Postprocessor Error", f"Postprocessor exited with code {exit_code}.")
                self.lbl_status.setText("Export failed.")

        self._run_postprocessor_async(script_path, self.last_file_path, out_file, on_finished)

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

    @staticmethod
    def _detect_gcode_format(file_path):
        """Auto-detect G-code format by scanning the first 200 lines.

        Returns:
            'nc'    — if M3/M5 spindle commands or standalone S lines found
            'prusa' — if E parameter in G1 lines or slicer comments found
        """
        has_m3_m5 = False
        has_standalone_s = False
        has_e_param = False
        has_slicer_comment = False

        try:
            import re as _re
            with open(file_path, 'r', encoding='utf-8') as f:
                for i, line in enumerate(f):
                    if i >= 200:
                        break
                    stripped = line.strip()
                    if not stripped:
                        continue

                    if stripped.startswith('M3') and (len(stripped) == 2 or not stripped[2].isdigit()):
                        has_m3_m5 = True
                    if stripped == 'M5' or stripped.startswith('M5 '):
                        has_m3_m5 = True
                    if _re.match(r'^S[\d.]+\s*$', stripped):
                        has_standalone_s = True
                    if stripped.startswith(';TYPE:') or stripped.startswith(';LAYER_CHANGE'):
                        has_slicer_comment = True
                    if stripped.startswith('G1') and ' E' in stripped:
                        has_e_param = True
        except Exception:
            pass

        # Decision
        if has_m3_m5 or has_standalone_s:
            return 'nc'
        if has_e_param or has_slicer_comment:
            return 'prusa'

        # Fallback by extension
        _, ext = os.path.splitext(file_path)
        if ext.lower() == '.nc':
            return 'nc'
        return 'prusa'

    def populate_importers(self):
        """Scan Importer/ dir, dynamically load each plugin, fill combobox."""
        import importlib.util

        self.combo_importer.clear()
        self._importer_modules = []

        importer_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Importer")
        if not os.path.isdir(importer_dir):
            return

        for fname in sorted(f for f in os.listdir(importer_dir) if f.endswith('.py')):
            script_path = os.path.join(importer_dir, fname)
            try:
                spec = importlib.util.spec_from_file_location(fname[:-3], script_path)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                label = mod.get_label() if hasattr(mod, 'get_label') else fname
                self.combo_importer.addItem(f"{label}  [{fname}]", userData=fname)
                self._importer_modules.append(mod)
            except Exception as exc:
                print(f"[Importer] Failed to load {fname}: {exc}")

        self.btn_run_import.setEnabled(
            bool(self._importer_modules) and bool(self._import_gcode_path))

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
        for i, mod in enumerate(self._importer_modules):
            try:
                if hasattr(mod, 'can_handle') and mod.can_handle(file_path):
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

        self.btn_run_import.setEnabled(bool(self._importer_modules))

    def _run_import(self):
        """Invoke the selected plugin's run_import() in a background thread."""
        if not self._import_gcode_path:
            QMessageBox.warning(self, "No File", "Select a G-code file first.")
            return

        idx = self.combo_importer.currentIndex()
        if idx < 0 or idx >= len(self._importer_modules):
            QMessageBox.warning(self, "No Importer", "No importer selected.")
            return

        mod = self._importer_modules[idx]
        fname = self.combo_importer.itemData(idx) or self.combo_importer.currentText()

        app_dir = os.path.dirname(os.path.abspath(__file__))
        csv_dir = os.path.join(app_dir, "CSV")
        os.makedirs(csv_dir, exist_ok=True)
        name_only, _ = os.path.splitext(os.path.basename(self._import_gcode_path))
        csv_out_path = os.path.join(csv_dir, name_only + ".csv")

        self._pending_csv_out = csv_out_path
        self.lbl_import_status.setText(f"Running {fname}...")
        self.btn_run_import.setEnabled(False)
        self.btn_select_gcode.setEnabled(False)

        self._import_thread = ImporterThread(mod, self._import_gcode_path, csv_out_path)
        self._import_thread.finished_signal.connect(self._on_import_finished)
        self._import_thread.start()

    def _on_import_finished(self, success: bool, payload: str):
        """Handle ImporterThread completion."""
        self.btn_run_import.setEnabled(True)
        self.btn_select_gcode.setEnabled(True)
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
        """Open a file dialog to select and load a URDF robot model."""
        urdf_dir = os.path.dirname(self.last_urdf_path) if self.last_urdf_path else os.getcwd()
        file_path, _ = QFileDialog.getOpenFileName(self, "Select URDF Robot File", urdf_dir, "URDF Files (*.urdf);;All Files (*)")
        if not file_path:
            return
        self.load_urdf(file_path)

    def load_urdf(self, file_path):
        """Load a URDF robot model and add its meshes to the scene."""
        if not os.path.exists(file_path):
            self.lbl_status.setText("URDF file not found.")
            return

        self.lbl_status.setText(f"Loading Robot: {os.path.basename(file_path)}...")
        QApplication.processEvents()

        try:
            self.robot_sim.load_robot(file_path)
            self.last_urdf_path = file_path

            for actor in self.robot_actors.values():
                self.plotter.remove_actor(actor)
            self.robot_actors.clear()

            for link_name, mesh in self.robot_sim.link_meshes.items():
                actor = self.plotter.add_mesh(mesh, color='#c0c0c0', show_edges=True)
                actor.SetVisibility(self.show_robot)
                self.robot_actors[link_name] = actor

            self.lbl_status.setText("Robot loaded successfully.")
            self.plotter.reset_camera()

            # Enable trajectory test button if data is also loaded
            if self.points_xyz is not None and len(self.points_xyz) > 0:
                self.btn_test_traj.setEnabled(True)
                self.color_strip.set_statuses(None)

            self.update_plot()

        except Exception as e:
            QMessageBox.critical(self, "Error Loading Robot", f"Failed to load URDF:\n{e}")
            self.lbl_status.setText("Robot load failed.")

    def load_file(self, file_path):
        """Start asynchronous loading of a CSV trajectory file."""
        if not os.path.exists(file_path):
            self.lbl_status.setText("Last saved file not found.")
            return

        self.last_file_path = file_path
        self.lbl_status.setText(f"Loading {os.path.basename(file_path)}...")
        self.h_slider.setEnabled(False)
        self.v_slider.setEnabled(False)

        self.loader_thread = DataLoaderThread(file_path)
        self.loader_thread.finished_signal.connect(self.on_load_finished)
        self.loader_thread.error_signal.connect(self.on_load_error)
        self.loader_thread.start()

    def on_load_error(self, err_msg):
        """Handle CSV loading errors."""
        QMessageBox.critical(self, "Error Loading File", err_msg)
        self.lbl_status.setText("Load error.")

    def on_load_finished(self, points_xyz, orientations_abc, colors_rgb, layer_ends, max_layer, estimated_time_s, estimated_weight_g):
        """Handle successful CSV load: store data and update UI."""
        if points_xyz is None or len(points_xyz) == 0:
            QMessageBox.warning(self, "No Data", "No valid points found in CSV.")
            self.lbl_status.setText("Load failed.")
            return

        self.points_xyz = points_xyz
        self._original_points_xyz = points_xyz.copy()

        # Reset edit transform when new data is loaded
        self.spin_edit_x.setValue(0.0)
        self.spin_edit_y.setValue(0.0)
        self.spin_edit_z_rot.setValue(0.0)
        self.lbl_edit_status.setText("")
        self.orientations_abc = orientations_abc
        self.colors_rgb = colors_rgb
        self.layer_end_indices = layer_ends
        self.max_layer = max_layer

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

        if num_pts > 0 and self.robot_sim.urdf_model:
            self.btn_test_traj.setEnabled(True)
            self.color_strip.set_statuses(None)

    # ------------------------------------------------------------------
    # Slider event handlers (with debounce)
    # ------------------------------------------------------------------

    def on_thickness_changed(self, val):
        """Handle extrusion width slider change."""
        self.print_thickness = float(val) / 10.0
        self.lbl_thickness.setText(f"Extrusion Width:\n{self.print_thickness:.1f} mm")
        if self.points_xyz is not None:
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
        """Request a debounced plot update — restarts the timer on each call."""
        self._pending_tube_render = True
        self._render_timer.start()

    def _deferred_update_plot(self):
        """Called by debounce timer — performs the actual rendering."""
        if self._pending_tube_render:
            self._pending_tube_render = False
            self.update_plot()

    def update_plot(self):
        """Rebuild the 3D path visualization up to current_step.

        Rendering is suppressed during the actor swap to prevent flickering.
        VTK only renders the final composited frame, never an intermediate
        state where old actors have been removed but new ones not yet added.
        """
        # Suppress rendering to avoid flicker during actor swap
        self.plotter.suppress_rendering = True
        try:
            self._rebuild_actors()
        finally:
            self.plotter.suppress_rendering = False
            self.plotter.render()

    def _rebuild_actors(self):
        """Internal: remove old actors and create new ones (called while rendering is suppressed)."""
        if self.points_xyz is None or self.current_step < 2:
            for attr in ('path_actor_print', 'path_actor_travel', 'tool_actor'):
                actor = getattr(self, attr, None)
                if actor:
                    self.plotter.remove_actor(actor)
                    setattr(self, attr, None)
            return

        num_segments = self.current_step - 1
        sub_colors = self.colors_rgb[:num_segments]
        sub_points = self.points_xyz[:self.current_step]

        # Identify print vs travel/retract segments by color match
        is_print_move = np.all(sub_colors == COLOR_PRINT, axis=1)

        # --- Actor 1: Print moves (3D tubes) ---
        print_idx = np.where(is_print_move)[0]
        if self.path_actor_print:
            self.plotter.remove_actor(self.path_actor_print)
            self.path_actor_print = None

        if len(print_idx) > 0:
            print_lines = np.empty((len(print_idx), 3), dtype=np.int32)
            print_lines[:, 0] = 2
            print_lines[:, 1] = print_idx
            print_lines[:, 2] = print_idx + 1
            mesh_print = pv.PolyData()
            mesh_print.points = sub_points
            mesh_print.lines = print_lines.ravel()
            mesh_print.cell_data["Colors"] = sub_colors[print_idx]

            try:
                tubes_print = mesh_print.tube(radius=self.print_thickness / 2.0, n_sides=8)
                self.path_actor_print = self.plotter.add_mesh(
                    tubes_print, scalars="Colors", rgb=True, reset_camera=False
                )
            except Exception:
                self.path_actor_print = self.plotter.add_mesh(
                    mesh_print, scalars="Colors", rgb=True,
                    line_width=max(1.0, self.print_thickness), reset_camera=False
                )

        # --- Actor 2: Travel/Retract moves (thin lines) ---
        travel_idx = np.where(~is_print_move)[0]
        if self.path_actor_travel:
            self.plotter.remove_actor(self.path_actor_travel)
            self.path_actor_travel = None

        if len(travel_idx) > 0:
            travel_lines = np.empty((len(travel_idx), 3), dtype=np.int32)
            travel_lines[:, 0] = 2
            travel_lines[:, 1] = travel_idx
            travel_lines[:, 2] = travel_idx + 1
            mesh_travel = pv.PolyData()
            mesh_travel.points = sub_points
            mesh_travel.lines = travel_lines.ravel()
            mesh_travel.cell_data["Colors"] = sub_colors[travel_idx]

            self.path_actor_travel = self.plotter.add_mesh(
                mesh_travel, scalars="Colors", rgb=True,
                line_width=1.0, reset_camera=False
            )

        # --- Toolhead indicator sphere ---
        if self.tool_actor:
            self.plotter.remove_actor(self.tool_actor)

        last_pt = self.points_xyz[self.current_step - 1]
        tool_mesh = pv.Sphere(radius=self.print_thickness * 0.6, center=last_pt)
        self.tool_actor = self.plotter.add_mesh(tool_mesh, color='cyan', reset_camera=False)

        # --- Robot IK follow ---
        self._update_robot_ik(last_pt)

    def _update_robot_ik(self, target_pt):
        """Compute IK for the robot to follow the target point and update visuals."""
        if not self.robot_sim.urdf_model or not self.show_robot:
            return

        try:
            t_point = np.eye(4)
            t_point[0:3, 3] = target_pt

            t_base = kuka_base_to_matrix(self.base_x, self.base_y, self.base_z, self.base_a, self.base_b, self.base_c)
            t_base_inv = np.linalg.inv(t_base)

            t_tool = kuka_base_to_matrix(self.tool_x, self.tool_y, self.tool_z, self.tool_a, self.tool_b, self.tool_c)
            t_tool_inv = np.linalg.inv(t_tool)

            t_tcp_target = t_base @ t_point
            t_flange_target = t_tcp_target @ t_tool_inv

            target_pos = t_flange_target[0:3, 3] / 1000.0
            target_ori = t_flange_target[0:3, 0:3]

            ik_solution = self.robot_sim.calculate_ik(target_pos, target_orientation=target_ori)

            if ik_solution is not None:
                limit_violations = self.robot_sim.check_limits(ik_solution)
                singularities = self.robot_sim.check_singularities(ik_solution)

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

        self.btn_test_traj.setEnabled(False)
        self.btn_load.setEnabled(False)
        self.btn_load_urdf.setEnabled(False)
        self.h_slider.setEnabled(False)
        self.v_slider.setEnabled(False)
        self.lbl_status.setText("Testing trajectory... This might take a while.")

        base_params = (self.base_x, self.base_y, self.base_z, self.base_a, self.base_b, self.base_c)
        tool_params = (self.tool_x, self.tool_y, self.tool_z, self.tool_a, self.tool_b, self.tool_c)

        self.traj_thread = TrajectoryTestThread(self.points_xyz, self.robot_sim, base_params, tool_params)
        self.traj_thread.progress_signal.connect(self.on_traj_test_progress)
        self.traj_thread.finished_signal.connect(self.on_traj_test_finished)
        self.traj_thread.start()

    def on_traj_test_progress(self, current, total):
        """Update status label with trajectory test progress."""
        self.lbl_status.setText(f"Testing trajectory: {current} / {total} points checked")

    def on_traj_test_finished(self, statuses):
        """Handle trajectory test completion."""
        self.lbl_status.setText("Test complete.")
        self.btn_test_traj.setEnabled(True)
        self.btn_load.setEnabled(True)
        self.btn_load_urdf.setEnabled(True)
        self.h_slider.setEnabled(True)
        self.v_slider.setEnabled(True)

        if statuses is not None:
            self.color_strip.set_statuses(statuses)

    # ------------------------------------------------------------------
    # Settings persistence
    # ------------------------------------------------------------------

    def load_settings(self):
        """Load application settings from JSON file."""
        if not os.path.exists(self.settings_file):
            return

        try:
            with open(self.settings_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
                self.print_thickness = config.get("print_thickness", 1.0)
                self.last_file_path = config.get("last_file_path", "")
                self.last_postprocessor = config.get("last_postprocessor", "")
                self.last_urdf_path = config.get("last_urdf_path", "")

                self.base_x = config.get("base_x", 0.0)
                self.base_y = config.get("base_y", 0.0)
                self.base_z = config.get("base_z", 0.0)
                self.base_a = config.get("base_a", 0.0)
                self.base_b = config.get("base_b", 0.0)
                self.base_c = config.get("base_c", 0.0)

                self.tool_x = config.get("tool_x", 0.0)
                self.tool_y = config.get("tool_y", 0.0)
                self.tool_z = config.get("tool_z", 0.0)
                self.tool_a = config.get("tool_a", 0.0)
                self.tool_b = config.get("tool_b", 0.0)
                self.tool_c = config.get("tool_c", 0.0)

                self.table_x1 = config.get("table_x1", 0.0)
                self.table_y1 = config.get("table_y1", 0.0)
                self.table_x2 = config.get("table_x2", 550.0)
                self.table_y2 = config.get("table_y2", 650.0)
                self.show_table = config.get("show_table", True)
                self.show_robot = config.get("show_robot", True)

                if self.last_postprocessor:
                    idx = self.combo_postprocessor.findText(self.last_postprocessor)
                    if idx >= 0:
                        self.combo_postprocessor.setCurrentIndex(idx)

        except Exception as e:
            print(f"Error loading settings: {e}")

    def save_settings(self):
        """Save application settings to JSON file."""
        config = {
            "print_thickness": self.print_thickness,
            "last_file_path": self.last_file_path,
            "last_postprocessor": self.combo_postprocessor.currentText(),
            "last_urdf_path": self.last_urdf_path,
            "base_x": self.base_x,
            "base_y": self.base_y,
            "base_z": self.base_z,
            "base_a": self.base_a,
            "base_b": self.base_b,
            "base_c": self.base_c,
            "tool_x": self.tool_x,
            "tool_y": self.tool_y,
            "tool_z": self.tool_z,
            "tool_a": self.tool_a,
            "tool_b": self.tool_b,
            "tool_c": self.tool_c,
            "table_x1": self.table_x1,
            "table_y1": self.table_y1,
            "table_x2": self.table_x2,
            "table_y2": self.table_y2,
            "show_table": self.show_table,
            "show_robot": self.show_robot
        }
        try:
            with open(self.settings_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=4)
        except Exception as e:
            print(f"Error saving settings: {e}")

    def closeEvent(self, event):
        """Save settings before closing the application."""
        self.save_settings()
        event.accept()


if __name__ == "__main__":
    multiprocessing.freeze_support()
    app = QApplication(sys.argv)

    # ---------------------------------------------------------------------------
    # Splash screen — shown IMMEDIATELY, before any heavy imports.
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
    # Heavy imports — happen here while the splash is visible.
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
