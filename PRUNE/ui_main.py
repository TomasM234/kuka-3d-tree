import logging
from pathlib import Path

import pyvista as pv
if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parent.parent
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    __package__ = "PRUNE"

from pyvistaqt import QtInteractor
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDockWidget,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from PRUNE import processor

logger = logging.getLogger(__name__)


class ProcessorWorker(QThread):
    finished = pyqtSignal(object)
    error = pyqtSignal(str)

    def __init__(self, func, *args, **kwargs):
        super().__init__()
        self.func = func
        self.args = args
        self.kwargs = kwargs

    def run(self):
        try:
            result = self.func(*self.args, **self.kwargs)
            self.finished.emit(result)
        except Exception as e:
            import traceback
            logger.error("Worker error:\n" + traceback.format_exc())
            self.error.emit(str(e))


class PruneMainWindow(QMainWindow):
    """Main window for the PRUNE CAD processor application."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("P.R.U.N.E. - Polygonal Reduction Utility")
        self.resize(1200, 800)

        # State
        self.current_mesh: pv.PolyData | None = None
        self.original_mesh: pv.PolyData | None = None
        self.mesh_actor = None
        self.original_actor = None
        self.mesh_color = "lightblue"
        self.worker = None
        
        # Processor module instances will be called here
        
        self._create_main_layout()
        self._create_dock_panels()
        self._create_viewport()

        # Try to show something on start
        self._reset_view()

    def _create_main_layout(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        menubar = self.menuBar()
        self.menu_file = menubar.addMenu("File")
        
        # Add basic file actions
        action_open = self.menu_file.addAction("Open Model...")
        action_open.triggered.connect(self.on_open_file)
        
        action_save = self.menu_file.addAction("Export STL...")
        action_save.triggered.connect(self.on_export_file)

    def _create_dock_panels(self):
        # Tools panel on the left layout
        self.dock_tools = QDockWidget("PRUNE Tools", self)
        self.dock_tools.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea)
        self.dock_tools.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetFloatable | QDockWidget.DockWidgetFeature.DockWidgetMovable)
        
        # Container for the dock
        self.tools_container = QWidget()
        self.tools_layout = QVBoxLayout(self.tools_container)
        self.tools_layout.setContentsMargins(10, 10, 10, 10)
        self.tools_layout.setSpacing(10)
        
        # 1. IO Section
        self.tools_layout.addWidget(QLabel("<b>FILE I/O</b>"))
        self.btn_open = QPushButton("Open CAD file (STL, OBJ, STP)...")
        self.btn_open.clicked.connect(self.on_open_file)
        self.tools_layout.addWidget(self.btn_open)
        
        self.btn_export = QPushButton("Export as STL...")
        self.btn_export.clicked.connect(self.on_export_file)
        self.btn_export.setEnabled(False)
        self.tools_layout.addWidget(self.btn_export)
        
        self.lbl_file_info = QLabel("No file loaded.")
        self.lbl_file_info.setWordWrap(True)
        self.tools_layout.addWidget(self.lbl_file_info)
        
        self.tools_layout.addSpacing(15)
        
        # 2. Simplification Section
        self.tools_layout.addWidget(QLabel("<b>MESH DECIMATION</b>"))
        self.tools_layout.addWidget(QLabel("Reduce the number of triangles in the mesh."))
        
        decimation_layout = QHBoxLayout()
        self.lbl_decimate_target = QLabel("Target Reduction: 50%")
        decimation_layout.addWidget(self.lbl_decimate_target)
        
        self.slider_decimate = QSlider(Qt.Orientation.Horizontal)
        self.slider_decimate.setRange(1, 99)
        self.slider_decimate.setValue(50)
        self.slider_decimate.valueChanged.connect(lambda v: self.lbl_decimate_target.setText(f"Target Reduction: {v}%"))
        decimation_layout.addWidget(self.slider_decimate)
        self.tools_layout.addLayout(decimation_layout)
        
        self.btn_decimate = QPushButton("Apply Decimation")
        self.btn_decimate.clicked.connect(self.on_decimate)
        self.btn_decimate.setEnabled(False)
        self.tools_layout.addWidget(self.btn_decimate)
        
        self.tools_layout.addSpacing(15)
        
        # 3. Wrapping Section
        self.tools_layout.addWidget(QLabel("<b>ENVELOPES</b>"))
        self.tools_layout.addWidget(QLabel("Create a safe collision wrapper around the object."))
        
        self.btn_convex_hull = QPushButton("Generate Convex Hull")
        self.btn_convex_hull.clicked.connect(self.on_convex_hull)
        self.btn_convex_hull.setEnabled(False)
        self.tools_layout.addWidget(self.btn_convex_hull)
        
        self.tools_layout.addSpacing(5)
        
        wrap_layout = QHBoxLayout()
        wrap_layout.addWidget(QLabel("Pitch (mm):"))
        self.spin_voxel_pitch = QSpinBox()
        self.spin_voxel_pitch.setRange(1, 100)
        self.spin_voxel_pitch.setValue(5)
        wrap_layout.addWidget(self.spin_voxel_pitch)
        self.tools_layout.addLayout(wrap_layout)
        
        self.btn_shrinkwrap = QPushButton("Morphological Shrinkwrap")
        self.btn_shrinkwrap.clicked.connect(self.on_shrinkwrap)
        self.btn_shrinkwrap.setEnabled(False)
        self.tools_layout.addWidget(self.btn_shrinkwrap)
        
        self.check_show_original = QCheckBox("Show Original Mesh (Gray)")
        self.check_show_original.setChecked(True)
        self.check_show_original.stateChanged.connect(self._update_plot)
        self.tools_layout.addWidget(self.check_show_original)
        
        opacity_layout = QHBoxLayout()
        self.lbl_opacity = QLabel("Envelope Opacity: 50%")
        opacity_layout.addWidget(self.lbl_opacity)
        self.slider_opacity = QSlider(Qt.Orientation.Horizontal)
        self.slider_opacity.setRange(10, 100)
        self.slider_opacity.setValue(50)
        self.slider_opacity.valueChanged.connect(self.on_opacity_change)
        opacity_layout.addWidget(self.slider_opacity)
        self.tools_layout.addLayout(opacity_layout)
        
        self.tools_layout.addSpacing(15)
        
        # 4. View controls
        self.tools_layout.addWidget(QLabel("<b>VIEW & RESET</b>"))
        self.btn_reset_mesh = QPushButton("Restore Original Mesh")
        self.btn_reset_mesh.clicked.connect(self.on_reset_mesh)
        self.btn_reset_mesh.setEnabled(False)
        self.tools_layout.addWidget(self.btn_reset_mesh)
        
        self.check_show_edges = QCheckBox("Show Mesh Edges (Wireframe)")
        self.check_show_edges.setChecked(False)
        self.check_show_edges.stateChanged.connect(self._update_plot)
        self.tools_layout.addWidget(self.check_show_edges)
        
        self.tools_layout.addStretch()
        
        self.dock_tools.setWidget(self.tools_container)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.dock_tools)

    def _create_viewport(self):
        self.plotter = QtInteractor(self.central_widget)
        self.plotter.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.main_layout.addWidget(self.plotter.interactor, stretch=1)
        self.plotter.add_axes()

    def _reset_view(self):
        self.plotter.set_background("white")
        self.plotter.reset_camera()

    def on_opacity_change(self, value):
        self.lbl_opacity.setText(f"Envelope Opacity: {value}%")
        self._update_plot()

    def _update_plot(self):
        """Redraws the mesh in the pyvista plotter according to current state."""
        show_orig = self.check_show_original.isChecked()
        if self.original_mesh is not None and show_orig:
            if self.original_actor is None:
                self.original_actor = self.plotter.add_mesh(
                    self.original_mesh, 
                    color="gray", 
                    show_edges=False,
                    opacity=1.0,
                )
            else:
                self.original_actor.SetVisibility(True)
        elif self.original_actor is not None:
            self.original_actor.SetVisibility(False)
            
        # Handle Current Mesh
        if self.current_mesh is not None:
            opacity = self.slider_opacity.value() / 100.0
            show_edges = self.check_show_edges.isChecked()
            
            if self.mesh_actor is None:
                self.mesh_actor = self.plotter.add_mesh(
                    self.current_mesh, 
                    color=self.mesh_color, 
                    show_edges=show_edges,
                    edge_color="black",
                    opacity=opacity,
                )
            else:
                self.mesh_actor.mapper.dataset.copy_from(self.current_mesh)
                self.mesh_actor.prop.show_edges = show_edges
                self.mesh_actor.prop.opacity = opacity
        elif self.mesh_actor is not None:
            self.plotter.remove_actor(self.mesh_actor)
            self.mesh_actor = None

        self._update_stats()

    def _update_stats(self):
        if self.current_mesh is None:
            self.lbl_file_info.setText("No file loaded.")
        else:
            n_points = self.current_mesh.n_points
            n_cells = self.current_mesh.n_cells
            self.lbl_file_info.setText(f"Current Mesh:\nVertices: {n_points:,}\nTriangles: {n_cells:,}")

    def _run_in_background(self, func, *args, success_callback=None, error_context="Processing Error", **kwargs):
        if self.worker is not None and self.worker.isRunning():
            return
            
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        self.dock_tools.setEnabled(False)
        
        self.worker = ProcessorWorker(func, *args, **kwargs)
        
        if success_callback:
            self.worker.finished.connect(success_callback)
            
        self.worker.error.connect(lambda e: self._handle_worker_error(e, error_context))
        
        self.worker.finished.connect(self._cleanup_worker)
        self.worker.error.connect(self._cleanup_worker)
        
        self.worker.start()

    def _cleanup_worker(self, _=None):
        self.dock_tools.setEnabled(True)
        QApplication.restoreOverrideCursor()
        self.worker = None

    def _handle_worker_error(self, error_msg, context):
        QMessageBox.critical(self, context, f"{context}:\n{error_msg}")

    def on_open_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open CAD/Mesh File", "", "Supported Files (*.stl *.obj *.stp *.step);;All Files (*.*)"
        )
        if not file_path:
            return
            
        def on_success(mesh):
            self.original_mesh = mesh.copy()
            self.current_mesh = mesh.copy()
            self._update_plot()
            self.plotter.reset_camera()
            
            # Enable buttons
            self.btn_export.setEnabled(True)
            self.btn_decimate.setEnabled(True)
            self.btn_convex_hull.setEnabled(True)
            self.btn_shrinkwrap.setEnabled(True)
            self.btn_reset_mesh.setEnabled(True)
            
        self._run_in_background(
            processor.load_mesh, file_path, 
            success_callback=on_success, 
            error_context="Error Loading File"
        )

    def on_export_file(self):
        if self.current_mesh is None:
            return
            
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Export STL", "", "STL Files (*.stl)"
        )
        if not file_path:
            return
            
        try:
            processor.save_stl(self.current_mesh, file_path)
            QMessageBox.information(self, "Success", f"Successfully exported to:\n{file_path}")
        except Exception as e:
            logger.exception("Failed to save file.")
            QMessageBox.critical(self, "Error Saving File", f"Could not save {file_path}:\n{e}")

    def on_decimate(self):
        if self.current_mesh is None:
            return
        
        reduction = self.slider_decimate.value() / 100.0
        
        def on_success(mesh):
            self.current_mesh = mesh
            self._update_plot()
            
        self._run_in_background(
            processor.apply_decimation, self.current_mesh, reduction,
            success_callback=on_success,
            error_context="Decimation Error"
        )

    def on_convex_hull(self):
        if self.current_mesh is None:
            return
            
        def on_success(mesh):
            self.current_mesh = mesh
            self._update_plot()
            
        self._run_in_background(
            processor.apply_convex_hull, self.current_mesh,
            success_callback=on_success,
            error_context="Convex Hull Error"
        )

    def on_shrinkwrap(self):
        if self.current_mesh is None:
            return
            
        pitch = self.spin_voxel_pitch.value()
        
        def on_success(mesh):
            self.current_mesh = mesh
            self._update_plot()
            
        self._run_in_background(
            processor.apply_voxel_shrinkwrap, self.current_mesh, pitch,
            success_callback=on_success,
            error_context="Shrinkwrap Error"
        )

    def on_reset_mesh(self):
        if self.original_mesh is not None:
            self.current_mesh = self.original_mesh.copy()
            self._update_plot()
            self.plotter.reset_camera()
