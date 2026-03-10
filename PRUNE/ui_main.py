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
from PyQt6.QtCore import Qt
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
        self.mesh_color = "lightblue"
        
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

    def _update_plot(self):
        """Redraws the mesh in the pyvista plotter according to current state."""
        if self.mesh_actor:
            self.plotter.remove_actor(self.mesh_actor)
            self.mesh_actor = None
            
        if self.current_mesh is not None:
            self.mesh_actor = self.plotter.add_mesh(
                self.current_mesh, 
                color=self.mesh_color, 
                show_edges=self.check_show_edges.isChecked(),
                edge_color="black",
                opacity=1.0,
            )
            self.plotter.reset_camera()

        self._update_stats()

    def _update_stats(self):
        if self.current_mesh is None:
            self.lbl_file_info.setText("No file loaded.")
        else:
            n_points = self.current_mesh.n_points
            n_cells = self.current_mesh.n_cells
            self.lbl_file_info.setText(f"Current Mesh:\nVertices: {n_points:,}\nTriangles: {n_cells:,}")

    def on_open_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open CAD/Mesh File", "", "Supported Files (*.stl *.obj *.stp *.step);;All Files (*.*)"
        )
        if not file_path:
            return
            
        try:
            mesh = processor.load_mesh(file_path)
            self.original_mesh = mesh.copy()
            self.current_mesh = mesh.copy()
            self._update_plot()
            
            # Enable buttons
            self.btn_export.setEnabled(True)
            self.btn_decimate.setEnabled(True)
            self.btn_convex_hull.setEnabled(True)
            self.btn_shrinkwrap.setEnabled(True)
            self.btn_reset_mesh.setEnabled(True)
            
        except Exception as e:
            logger.exception("Failed to load file.")
            QMessageBox.critical(self, "Error Loading File", f"Could not load {file_path}:\n{e}")

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
        try:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            self.current_mesh = processor.apply_decimation(self.current_mesh, reduction)
            self._update_plot()
        except Exception as e:
            logger.exception("Decimation failed.")
            QMessageBox.critical(self, "Decimation Error", f"Failed to decimate mesh:\n{e}")
        finally:
            QApplication.restoreOverrideCursor()

    def on_convex_hull(self):
        if self.current_mesh is None:
            return
            
        try:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            self.current_mesh = processor.apply_convex_hull(self.current_mesh)
            self._update_plot()
        except Exception as e:
            logger.exception("Convex Hull failed.")
            QMessageBox.critical(self, "Convex Hull Error", f"Failed to generate convex hull:\n{e}")
        finally:
            QApplication.restoreOverrideCursor()

    def on_shrinkwrap(self):
        if self.current_mesh is None:
            return
            
        pitch = self.spin_voxel_pitch.value()
        try:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            self.current_mesh = processor.apply_voxel_shrinkwrap(self.current_mesh, pitch)
            self._update_plot()
        except Exception as e:
            logger.exception("Shrinkwrap failed.")
            QMessageBox.critical(self, "Shrinkwrap Error", f"Failed to generate shrinkwrap envelope:\n{e}")
        finally:
            QApplication.restoreOverrideCursor()

    def on_reset_mesh(self):
        if self.original_mesh is not None:
            self.current_mesh = self.original_mesh.copy()
            self._update_plot()
