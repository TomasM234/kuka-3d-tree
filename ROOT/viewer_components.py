import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal, QRectF
from PyQt6.QtGui import QColor, QPainter
from PyQt6.QtWidgets import QWidget

from .error_reporting import get_logger
from .trajectory_schema import load_trajectory_csv
from .trajectory_test_lib import TrajectoryTestConfig, run_trajectory_test_parallel


logger = get_logger(__name__)


class ColorStripWidget(QWidget):
    """Thin horizontal bar showing trajectory test results as colored segments."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(12)
        self.statuses = None
        self.colors = {
            0: QColor(0, 220, 0),
            1: QColor(255, 200, 0),
            2: QColor(255, 0, 0),
            3: QColor(100, 100, 100),
        }

    def set_statuses(self, statuses):
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

    progress_signal = pyqtSignal(object)
    error_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(object)

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
                seed_templates=self.seed_templates,
            )
            statuses = run_trajectory_test_parallel(
                self.points_xyz,
                self.orientations_abc,
                config,
                progress_callback=self.progress_signal.emit,
            )
        except Exception as exc:
            logger.exception("Trajectory test worker failed")
            self.error_signal.emit(str(exc))
            return

        self.finished_signal.emit(statuses)


class DataLoaderThread(QThread):
    """Background thread that parses a CSV trajectory file into numpy arrays."""

    finished_signal = pyqtSignal(object, object, object, object, int, float, float)
    error_signal = pyqtSignal(str)

    def __init__(self, file_path):
        super().__init__()
        self.file_path = file_path

    def run(self):
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
        except Exception as exc:
            logger.exception("CSV loader thread failed for %s", self.file_path)
            self.error_signal.emit(str(exc))


class ImporterThread(QThread):
    """Background thread that calls a plugin importer's run_import() function."""

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
            logger.exception("Importer thread failed for %s", self.input_path)
            self.finished_signal.emit(False, str(exc))
