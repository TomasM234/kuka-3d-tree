from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QAction, QIcon
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDockWidget,
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
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from .app_paths import APPDATA_DIR, EXTRUDER_DIR
from .viewer_components import ColorStripWidget
from .viewer_ik_controller import IK_CONFIGS


class ViewerUiTabsMixin:
    def _create_main_layout(self):
        """Create the central viewport container and top-level panel menus."""
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        self._panel_containers = []
        self._dock_panels = {}
        self._last_dock_widget = None

        menubar = self.menuBar()
        self.menu_file = menubar.addMenu("File")
        self.menu_workplace = menubar.addMenu("Workplace")
        self.menu_tools = menubar.addMenu("Tools")
        self.menu_perspectives = menubar.addMenu("Perspectives")

        self.perspective_toolbar = QToolBar("Perspectives", self)
        self.perspective_toolbar.setObjectName("perspective_toolbar")
        self.perspective_toolbar.setMovable(False)
        self.perspective_toolbar.setFloatable(False)
        self.perspective_toolbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonIconOnly)
        self.perspective_toolbar.setIconSize(QSize(48, 48))
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, self.perspective_toolbar)

    def _register_panel_container(self, container):
        self._panel_containers.append(container)

    def _create_dock_panel(self, panel_key, title, menu, object_name):
        dock = QDockWidget(title, self)
        dock.setObjectName(object_name)
        dock.setAllowedAreas(
            Qt.DockWidgetArea.LeftDockWidgetArea |
            Qt.DockWidgetArea.RightDockWidgetArea
        )
        dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable |
            QDockWidget.DockWidgetFeature.DockWidgetFloatable |
            QDockWidget.DockWidgetFeature.DockWidgetClosable
        )

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        scroll.setWidget(container)
        dock.setWidget(scroll)

        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, dock)
        if self._last_dock_widget is not None:
            self.splitDockWidget(self._last_dock_widget, dock, Qt.Orientation.Vertical)
        self._last_dock_widget = dock
        self._dock_panels[panel_key] = dock
        self._register_panel_container(container)

        toggle_action = dock.toggleViewAction()
        toggle_action.setText(title)
        menu.addAction(toggle_action)
        return layout

    @staticmethod
    def _build_icon(file_name):
        icon_path = APPDATA_DIR / file_name
        if icon_path.exists():
            return QIcon(str(icon_path))
        return QIcon()

    def _activate_perspective(self, panel_keys):
        target = set(panel_keys)
        for key, dock in self._dock_panels.items():
            dock.setVisible(key in target)

    def _open_all_panels(self):
        for dock in self._dock_panels.values():
            dock.setVisible(True)

    def _close_all_panels(self):
        for dock in self._dock_panels.values():
            dock.setVisible(False)

    def _build_perspective_action(self, text, icon_file, callback):
        action = QAction(self._build_icon(icon_file), text, self)
        action.triggered.connect(callback)
        action.setToolTip(text)
        return action

    def _setup_perspectives(self):
        perspective_items = [
            ("Open files", "Open.png", ("project", "open_csv", "import", "display")),
            ("Export", "Export.png", ("export",)),
            ("Robot cell", "RobotCell.png", ("table",)),
            ("Extruder", "Extruder.png", ("extruder",)),
            ("Robot setup", "Robot.png", ("robot", "base", "tool")),
            ("Trajectory tools", "Trajectory.png", ("edit", "test", "actual_position")),
        ]

        self.menu_perspectives.clear()
        self.perspective_toolbar.clear()

        for label, icon_name, panel_keys in perspective_items:
            action = self._build_perspective_action(
                label,
                icon_name,
                lambda _, keys=panel_keys: self._activate_perspective(keys),
            )
            self.menu_perspectives.addAction(action)
            self.perspective_toolbar.addAction(action)

        self.menu_perspectives.addSeparator()
        self.perspective_toolbar.addSeparator()

        open_all_action = self._build_perspective_action(
            "Open all panels",
            "OpenPanels.png",
            self._open_all_panels,
        )
        close_all_action = self._build_perspective_action(
            "Close all panels",
            "ClosePanels.png",
            self._close_all_panels,
        )
        self.menu_perspectives.addAction(open_all_action)
        self.menu_perspectives.addAction(close_all_action)
        self.perspective_toolbar.addAction(open_all_action)
        self.perspective_toolbar.addAction(close_all_action)

    def _create_spinbox(self, min_val, max_val, suffix, value, callback):
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setSuffix(suffix)
        spin.setValue(value)
        spin.valueChanged.connect(callback)
        return spin

    def _create_general_tab(self):
        project_layout = self._create_dock_panel("project", "Project", self.menu_file, "dock_project")
        project_layout.addWidget(QLabel("<b>PROJECT</b>"))

        self.lbl_project_name = QLabel("\U0001F4C1 default")
        self.lbl_project_name.setWordWrap(True)
        project_layout.addWidget(self.lbl_project_name)

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
        project_layout.addLayout(proj_btn_row)
        project_layout.addStretch()

        open_csv_layout = self._create_dock_panel(
            "open_csv",
            "Open CSV trajectory",
            self.menu_file,
            "dock_open_csv_trajectory",
        )
        open_csv_layout.addWidget(QLabel("<b>TRAJECTORY (CSV)</b>"))

        self.btn_load = QPushButton("Open CSV File...")
        self.btn_load.clicked.connect(self.load_file_dialog)
        open_csv_layout.addWidget(self.btn_load)

        self.lbl_status = QLabel("Ready.")
        self.lbl_status.setWordWrap(True)
        open_csv_layout.addWidget(self.lbl_status)

        self.lbl_print_time = QLabel("")
        open_csv_layout.addWidget(self.lbl_print_time)

        self.lbl_print_weight = QLabel("")
        open_csv_layout.addWidget(self.lbl_print_weight)
        open_csv_layout.addStretch()

        display_layout = self._create_dock_panel(
            "display",
            "Display settings",
            self.menu_file,
            "dock_display_settings",
        )
        display_layout.addWidget(QLabel("<b>CAMERA</b>"))

        for label, slot in (
            ("Reset View (Iso)", self.view_isometric),
            ("Top View", self.view_top),
            ("Side View", self.view_side),
        ):
            button = QPushButton(label)
            button.clicked.connect(slot)
            display_layout.addWidget(button)

        display_layout.addSpacing(8)
        display_layout.addWidget(QLabel("<b>VISUALS</b>"))

        self.lbl_thickness = QLabel(f"Extrusion Width:\n{self.print_thickness:.1f} mm")
        display_layout.addWidget(self.lbl_thickness)

        self.slider_thick = QSlider(Qt.Orientation.Horizontal)
        self.slider_thick.setMinimum(2)
        self.slider_thick.setMaximum(50)
        self.slider_thick.setValue(int(self.print_thickness * 10))
        self.slider_thick.valueChanged.connect(self.on_thickness_changed)
        display_layout.addWidget(self.slider_thick)
        display_layout.addStretch()

        import_layout = self._create_dock_panel(
            "import",
            "Import trajectory",
            self.menu_file,
            "dock_import_trajectory",
        )
        import_layout.addWidget(QLabel("<b>IMPORT FROM G-CODE</b>"))

        self.lbl_import_file = QLabel("No file selected.")
        self.lbl_import_file.setWordWrap(True)
        import_layout.addWidget(self.lbl_import_file)

        self.btn_select_gcode = QPushButton("Select G-Code File...")
        self.btn_select_gcode.clicked.connect(self._select_gcode_for_import)
        import_layout.addWidget(self.btn_select_gcode)

        import_layout.addWidget(QLabel("Importer:"))
        self.combo_importer = QComboBox()
        import_layout.addWidget(self.combo_importer)

        self.btn_run_import = QPushButton("Import && Load")
        self.btn_run_import.clicked.connect(self._run_import)
        self.btn_run_import.setEnabled(False)
        import_layout.addWidget(self.btn_run_import)

        self.lbl_import_status = QLabel("")
        self.lbl_import_status.setWordWrap(True)
        import_layout.addWidget(self.lbl_import_status)

        self.populate_importers()
        import_layout.addStretch()

    def _create_workplace_tab(self):
        table_layout = self._create_dock_panel("table", "Table", self.menu_workplace, "dock_table")
        table_layout.addWidget(QLabel("<b>TABLE (Bed)</b>"))

        self.check_show_table = QCheckBox("Show Table")
        self.check_show_table.setChecked(self.show_table)
        self.check_show_table.stateChanged.connect(self.on_table_changed)
        table_layout.addWidget(self.check_show_table)

        self.table_form = QFormLayout()
        self.spin_table_x1 = self._create_spinbox(-5000, 5000, " mm", self.table_x1, self.on_table_changed)
        self.spin_table_y1 = self._create_spinbox(-5000, 5000, " mm", self.table_y1, self.on_table_changed)
        self.spin_table_x2 = self._create_spinbox(-5000, 5000, " mm", self.table_x2, self.on_table_changed)
        self.spin_table_y2 = self._create_spinbox(-5000, 5000, " mm", self.table_y2, self.on_table_changed)
        self.table_form.addRow("X1 (Corner 1):", self.spin_table_x1)
        self.table_form.addRow("Y1 (Corner 1):", self.spin_table_y1)
        self.table_form.addRow("X2 (Corner 2):", self.spin_table_x2)
        self.table_form.addRow("Y2 (Corner 2):", self.spin_table_y2)
        table_layout.addLayout(self.table_form)
        table_layout.addStretch()

        robot_layout = self._create_dock_panel("robot", "Robot", self.menu_workplace, "dock_robot")
        robot_layout.addWidget(QLabel("<b>ROBOT</b>"))

        self.lbl_robot_name = QLabel("No robot selected.")
        self.lbl_robot_name.setWordWrap(True)
        robot_layout.addWidget(self.lbl_robot_name)

        self.btn_load_urdf = QPushButton("Select Robot...")
        self.btn_load_urdf.clicked.connect(self.load_urdf_dialog)
        robot_layout.addWidget(self.btn_load_urdf)

        self.check_show_robot = QCheckBox("Show Robot")
        self.check_show_robot.setChecked(self.show_robot)
        self.check_show_robot.stateChanged.connect(self.on_robot_changed)
        robot_layout.addWidget(self.check_show_robot)
        robot_layout.addStretch()

        base_layout = self._create_dock_panel("base", "Base", self.menu_workplace, "dock_base")
        base_layout.addWidget(QLabel("<b>Robot Base Frame</b>"))

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
        base_layout.addLayout(self.base_form)
        base_layout.addStretch()

        tool_layout = self._create_dock_panel("tool", "Tool", self.menu_workplace, "dock_tool")
        tool_layout.addWidget(QLabel("<b>Robot Tool Frame</b>"))

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
        tool_layout.addLayout(self.tool_form)
        tool_layout.addStretch()

        extruder_layout = self._create_dock_panel("extruder", "Extruder", self.menu_workplace, "dock_extruder")
        extruder_layout.addWidget(QLabel("<b>EXTRUDER</b>"))
        
        extruder_layout.addWidget(QLabel("Extruder Model:"))
        self.combo_extruder_stl = QComboBox()
        self.combo_extruder_stl.addItem("None")
        if EXTRUDER_DIR.exists() and EXTRUDER_DIR.is_dir():
            files = [p.name for p in EXTRUDER_DIR.iterdir() if p.name.lower().endswith(".stl")]
            for f in sorted(files, key=str.lower):
                self.combo_extruder_stl.addItem(f)
        extruder_layout.addWidget(self.combo_extruder_stl)
        extruder_layout.addStretch()

    def _create_edit_tab(self):
        edit_layout = self._create_dock_panel(
            "edit",
            "Edit trajectory",
            self.menu_tools,
            "dock_edit_trajectory",
        )
        edit_layout.addWidget(QLabel("<b>TRANSFORM MODEL</b>"))
        edit_layout.addWidget(QLabel("Total offset from original data.\nRotation is around the model centroid."))
        edit_layout.addSpacing(10)

        edit_layout.addWidget(QLabel("<u>Shift X:</u>"))
        self.spin_edit_x = self._create_spinbox(-10000, 10000, " mm", 0.0, lambda: None)
        edit_layout.addWidget(self.spin_edit_x)
        x_layout = QHBoxLayout()
        for label, delta in (("-100", -100), ("-1", -1), ("+1", 1), ("+100", 100)):
            button = QPushButton(label)
            button.setFixedWidth(50)
            button.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_x, d))
            x_layout.addWidget(button)
        edit_layout.addLayout(x_layout)
        edit_layout.addSpacing(5)

        edit_layout.addWidget(QLabel("<u>Shift Y:</u>"))
        self.spin_edit_y = self._create_spinbox(-10000, 10000, " mm", 0.0, lambda: None)
        edit_layout.addWidget(self.spin_edit_y)
        y_layout = QHBoxLayout()
        for label, delta in (("-100", -100), ("-1", -1), ("+1", 1), ("+100", 100)):
            button = QPushButton(label)
            button.setFixedWidth(50)
            button.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_y, d))
            y_layout.addWidget(button)
        edit_layout.addLayout(y_layout)
        edit_layout.addSpacing(5)

        edit_layout.addWidget(QLabel("<u>Rotate Z:</u>"))
        self.spin_edit_z_rot = self._create_spinbox(-360, 360, " deg", 0.0, lambda: None)
        edit_layout.addWidget(self.spin_edit_z_rot)
        z_layout = QHBoxLayout()
        for label, delta in (("-5", -5), ("-1", -1), ("+1", 1), ("+5", 5)):
            button = QPushButton(label)
            button.setFixedWidth(50)
            button.clicked.connect(lambda _, d=delta: self._edit_increment(self.spin_edit_z_rot, d))
            z_layout.addWidget(button)
        edit_layout.addLayout(z_layout)
        edit_layout.addSpacing(10)

        self.btn_apply_edit = QPushButton("Apply Transform")
        self.btn_apply_edit.clicked.connect(self._apply_edit_transform)
        edit_layout.addWidget(self.btn_apply_edit)

        self.btn_reset_edit = QPushButton("Reset to Original")
        self.btn_reset_edit.clicked.connect(self._reset_edit_transform)
        edit_layout.addWidget(self.btn_reset_edit)

        edit_layout.addSpacing(15)

        self.btn_save_edit = QPushButton("Save to CSV")
        self.btn_save_edit.clicked.connect(self._save_edit_to_csv)
        edit_layout.addWidget(self.btn_save_edit)

        self.lbl_edit_status = QLabel("")
        self.lbl_edit_status.setWordWrap(True)
        edit_layout.addWidget(self.lbl_edit_status)

        edit_layout.addStretch()

        test_layout = self._create_dock_panel(
            "test",
            "Test trajectory",
            self.menu_tools,
            "dock_test_trajectory",
        )
        test_layout.addWidget(QLabel("<b>TRAJECTORY TEST</b>"))

        self.btn_test_traj = QPushButton("Test Trajectory")
        self.btn_test_traj.clicked.connect(self.run_trajectory_test)
        self.btn_test_traj.setEnabled(False)
        test_layout.addWidget(self.btn_test_traj)

        self.progress_traj_test = QProgressBar()
        self.progress_traj_test.setRange(0, 100)
        self.progress_traj_test.setValue(0)
        self.progress_traj_test.hide()
        test_layout.addWidget(self.progress_traj_test)

        self.lbl_traj_test_progress = QLabel("")
        self.lbl_traj_test_progress.setWordWrap(True)
        self.lbl_traj_test_progress.hide()
        test_layout.addWidget(self.lbl_traj_test_progress)

        self.lbl_traj_test_eta = QLabel("")
        self.lbl_traj_test_eta.setWordWrap(True)
        self.lbl_traj_test_eta.hide()
        test_layout.addWidget(self.lbl_traj_test_eta)
        test_layout.addStretch()

        pose_layout = self._create_dock_panel(
            "actual_position",
            "Actual robot position",
            self.menu_tools,
            "dock_actual_robot_position",
        )
        pose_layout.addWidget(QLabel("<b>CURRENT ROBOT POSE</b>"))
        pose_layout.addSpacing(6)
        pose_layout.addWidget(QLabel("<b>IK Start Configuration:</b>"))
        self.combo_ik_config = QComboBox()
        self.combo_ik_config.addItems(list(IK_CONFIGS.keys()))
        self.combo_ik_config.currentTextChanged.connect(self.on_ik_config_changed)
        pose_layout.addWidget(self.combo_ik_config)
        pose_layout.addSpacing(8)

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
            pose_layout.addWidget(label)

        pose_layout.addStretch()

    def _create_export_tab(self):
        export_layout = self._create_dock_panel(
            "export",
            "Export trajectory",
            self.menu_file,
            "dock_export_trajectory",
        )
        export_layout.addWidget(QLabel("<b>EXPORT & POSTPROCESS</b>"))

        self.combo_postprocessor = QComboBox()
        export_layout.addWidget(self.combo_postprocessor)

        self.btn_preview = QPushButton("Preview Script")
        self.btn_preview.clicked.connect(self.on_preview_clicked)
        export_layout.addWidget(self.btn_preview)

        self.btn_export = QPushButton("Export Generated Code")
        self.btn_export.clicked.connect(self.on_export_clicked)
        export_layout.addWidget(self.btn_export)

        self.populate_postprocessors()
        self._setup_perspectives()
        export_layout.addStretch()

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
