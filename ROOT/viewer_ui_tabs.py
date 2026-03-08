from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QProgressBar,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .viewer_components import ColorStripWidget
from .viewer_ik_controller import IK_CONFIGS


class ViewerUiTabsMixin:
    def _create_main_layout(self):
        """Create the top-level horizontal split: left panel + right viewport."""
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)

        self.left_scroll = QScrollArea()
        self.left_scroll.setWidgetResizable(True)
        self.left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.left_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.left_scroll.setFixedWidth(266)
        self.main_layout.addWidget(self.left_scroll)

        self.left_panel = QFrame()
        self.left_panel.setMinimumWidth(250)
        self.left_layout = QVBoxLayout(self.left_panel)
        self.left_panel.setFrameShape(QFrame.Shape.StyledPanel)
        self.left_scroll.setWidget(self.left_panel)

        self.tabs = QTabWidget()
        self.tabs.tabBar().setUsesScrollButtons(False)
        self.tabs.tabBar().setExpanding(True)
        self.left_layout.addWidget(self.tabs)

        self.left_layout.addSpacing(10)
        self.left_layout.addWidget(QLabel("<b>CURRENT ROBOT POSE</b>"))

        self.pose_grid = QVBoxLayout()
        self.pose_grid.setContentsMargins(5, 5, 5, 5)
        self.pose_grid.setSpacing(2)

        self.lbl_pose_xyz = QLabel("TCP mm: X:- Y:- Z:-")
        self.lbl_pose_abc = QLabel("TCP deg: A:- B:- C:-")
        self.lbl_pose_j123 = QLabel("J1-3 deg: A1:- A2:- A3:-")
        self.lbl_pose_j456 = QLabel("J4-6 deg: A4:- A5:- A6:-")

        for label in (self.lbl_pose_xyz, self.lbl_pose_abc, self.lbl_pose_j123, self.lbl_pose_j456):
            label.setStyleSheet("font-family: monospace; font-size: 11px;")
            label.setWordWrap(False)
            label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            label.setFixedHeight(label.fontMetrics().height() + 6)
            label.setToolTip(label.text())
            self.pose_grid.addWidget(label)

        self.left_layout.addLayout(self.pose_grid)

    def _create_spinbox(self, min_val, max_val, suffix, value, callback):
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setSuffix(suffix)
        spin.setValue(value)
        spin.valueChanged.connect(callback)
        return spin

    def _create_general_tab(self):
        self.tab_general = QWidget()
        self.layout_general = QVBoxLayout(self.tab_general)
        self.tabs.addTab(self.tab_general, "General")

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
        self.layout_general.addWidget(QLabel("<b>TRAJECTORY (CSV)</b>"))

        self.btn_load = QPushButton("Open CSV File...")
        self.btn_load.clicked.connect(self.load_file_dialog)
        self.layout_general.addWidget(self.btn_load)

        self.layout_general.addSpacing(12)
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

        self.lbl_status = QLabel("Ready.")
        self.lbl_status.setWordWrap(True)
        self.layout_general.addWidget(self.lbl_status)

        self.lbl_print_time = QLabel("")
        self.layout_general.addWidget(self.lbl_print_time)

        self.lbl_print_weight = QLabel("")
        self.layout_general.addWidget(self.lbl_print_weight)

        self.layout_general.addSpacing(12)
        self.layout_general.addWidget(QLabel("<b>CAMERA</b>"))

        for label, slot in (
            ("Reset View (Iso)", self.view_isometric),
            ("Top View", self.view_top),
            ("Side View", self.view_side),
        ):
            button = QPushButton(label)
            button.clicked.connect(slot)
            self.layout_general.addWidget(button)

        self.layout_general.addSpacing(12)
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
        self.tab_workplace = QWidget()
        self.layout_workplace = QVBoxLayout(self.tab_workplace)
        self.tabs.addTab(self.tab_workplace, "Workplace")

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

        self.progress_traj_test = QProgressBar()
        self.progress_traj_test.setRange(0, 100)
        self.progress_traj_test.setValue(0)
        self.progress_traj_test.hide()
        self.layout_workplace.addWidget(self.progress_traj_test)

        self.lbl_traj_test_progress = QLabel("")
        self.lbl_traj_test_progress.setWordWrap(True)
        self.lbl_traj_test_progress.hide()
        self.layout_workplace.addWidget(self.lbl_traj_test_progress)

        self.lbl_traj_test_eta = QLabel("")
        self.lbl_traj_test_eta.setWordWrap(True)
        self.lbl_traj_test_eta.hide()
        self.layout_workplace.addWidget(self.lbl_traj_test_eta)

        self.layout_workplace.addSpacing(5)
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

        self.layout_workplace.addWidget(QLabel("<u>Robot Base Frame:</u>"))
        self.base_form = QFormLayout()
        self.spin_base_x = self._create_spinbox(-5000, 5000, " mm", self.base_x, self.on_transform_changed)
        self.spin_base_y = self._create_spinbox(-5000, 5000, " mm", self.base_y, self.on_transform_changed)
        self.spin_base_z = self._create_spinbox(-5000, 5000, " mm", self.base_z, self.on_transform_changed)
        self.spin_base_a = self._create_spinbox(-360, 360, " deg", self.base_a, self.on_transform_changed)
        self.spin_base_b = self._create_spinbox(-360, 360, " deg", self.base_b, self.on_transform_changed)
        self.spin_base_c = self._create_spinbox(-360, 360, " deg", self.base_c, self.on_transform_changed)
        for label, spin in (
            ("X:", self.spin_base_x),
            ("Y:", self.spin_base_y),
            ("Z:", self.spin_base_z),
            ("A:", self.spin_base_a),
            ("B:", self.spin_base_b),
            ("C:", self.spin_base_c),
        ):
            self.base_form.addRow(label, spin)
        self.layout_workplace.addLayout(self.base_form)
        self.layout_workplace.addSpacing(10)

        self.layout_workplace.addWidget(QLabel("<u>Robot Tool Frame:</u>"))
        self.tool_form = QFormLayout()
        self.spin_tool_x = self._create_spinbox(-5000, 5000, " mm", self.tool_x, self.on_transform_changed)
        self.spin_tool_y = self._create_spinbox(-5000, 5000, " mm", self.tool_y, self.on_transform_changed)
        self.spin_tool_z = self._create_spinbox(-5000, 5000, " mm", self.tool_z, self.on_transform_changed)
        self.spin_tool_a = self._create_spinbox(-360, 360, " deg", self.tool_a, self.on_transform_changed)
        self.spin_tool_b = self._create_spinbox(-360, 360, " deg", self.tool_b, self.on_transform_changed)
        self.spin_tool_c = self._create_spinbox(-360, 360, " deg", self.tool_c, self.on_transform_changed)
        for label, spin in (
            ("X:", self.spin_tool_x),
            ("Y:", self.spin_tool_y),
            ("Z:", self.spin_tool_z),
            ("A:", self.spin_tool_a),
            ("B:", self.spin_tool_b),
            ("C:", self.spin_tool_c),
        ):
            self.tool_form.addRow(label, spin)
        self.layout_workplace.addLayout(self.tool_form)
        self.layout_workplace.addStretch()

    def _create_edit_tab(self):
        self.tab_edit = QWidget()
        self.layout_edit = QVBoxLayout(self.tab_edit)
        self.tabs.addTab(self.tab_edit, "Edit")

        self.layout_edit.addWidget(QLabel("<b>TRANSFORM MODEL</b>"))
        self.layout_edit.addWidget(QLabel("Total offset from original data.\nRotation is around the model centroid."))
        self.layout_edit.addSpacing(10)

        self.layout_edit.addWidget(QLabel("<u>Shift X:</u>"))
        self.spin_edit_x = self._create_spinbox(-10000, 10000, " mm", 0.0, lambda: None)
        self.layout_edit.addWidget(self.spin_edit_x)
        x_layout = QHBoxLayout()
        for label, delta in (("-100", -100), ("-1", -1), ("+1", 1), ("+100", 100)):
            button = QPushButton(label)
            button.setFixedWidth(50)
            button.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_x, d))
            x_layout.addWidget(button)
        self.layout_edit.addLayout(x_layout)
        self.layout_edit.addSpacing(5)

        self.layout_edit.addWidget(QLabel("<u>Shift Y:</u>"))
        self.spin_edit_y = self._create_spinbox(-10000, 10000, " mm", 0.0, lambda: None)
        self.layout_edit.addWidget(self.spin_edit_y)
        y_layout = QHBoxLayout()
        for label, delta in (("-100", -100), ("-1", -1), ("+1", 1), ("+100", 100)):
            button = QPushButton(label)
            button.setFixedWidth(50)
            button.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_y, d))
            y_layout.addWidget(button)
        self.layout_edit.addLayout(y_layout)
        self.layout_edit.addSpacing(5)

        self.layout_edit.addWidget(QLabel("<u>Rotate Z:</u>"))
        self.spin_edit_z_rot = self._create_spinbox(-360, 360, " deg", 0.0, lambda: None)
        self.layout_edit.addWidget(self.spin_edit_z_rot)
        z_layout = QHBoxLayout()
        for label, delta in (("-5", -5), ("-1", -1), ("+1", 1), ("+5", 5)):
            button = QPushButton(label)
            button.setFixedWidth(50)
            button.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_z_rot, d))
            z_layout.addWidget(button)
        self.layout_edit.addLayout(z_layout)
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

    def _create_export_tab(self):
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
        self.right_panel = QWidget()
        self.right_layout = QVBoxLayout(self.right_panel)
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.addWidget(self.right_panel, stretch=1)

        self.plot_h_layout = QHBoxLayout()
        self.right_layout.addLayout(self.plot_h_layout, stretch=1)

        self.plotter = self.qt_interactor_cls(self.central_widget)
        self.plotter.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.plot_h_layout.addWidget(self.plotter.interactor, stretch=1)

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
