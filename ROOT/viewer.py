if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parent.parent
    repo_root_str = str(repo_root)
    if repo_root_str not in sys.path:
        sys.path.insert(0, repo_root_str)
    __package__ = "ROOT"

    if __name__ == "__main__":
        from ROOT.viewer_app import main as _viewer_main

        raise SystemExit(_viewer_main(sys.argv[1:]))

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QMainWindow

from .app_paths import PROJECT_DIR, SETTINGS_FILE
from .project_config import ProjectConfig, ProjectConfigStore
from .task_controller import TaskController
from .trajectory_render import TrajectoryRenderer
from .viewer_edit_mixin import ViewerEditMixin
from .viewer_ik_controller import ViewerIkControllerMixin
from .viewer_io_mixin import ViewerIOMixin
from .viewer_project_mixin import ViewerProjectMixin
from .viewer_render_mixin import ViewerRenderMixin
from .viewer_runtime_mixin import ViewerRuntimeMixin
from .viewer_state_mixin import ViewerStateMixin
from .viewer_ui_tabs import ViewerUiTabsMixin


class RobotPathViewer(
    ViewerProjectMixin,
    ViewerIOMixin,
    ViewerEditMixin,
    ViewerRenderMixin,
    ViewerIkControllerMixin,
    ViewerRuntimeMixin,
    ViewerStateMixin,
    ViewerUiTabsMixin,
    QMainWindow,
):
    """Main application window for trajectory import, review, validation, and export."""

    def __init__(self, dependencies):
        super().__init__()
        self.setWindowTitle("Robot CSV 3D Viewer")
        self.resize(1300, 900)

        self.dependencies = dependencies
        self.pv = dependencies.pv_module
        self.qt_interactor_cls = dependencies.qt_interactor_cls
        self.robot_simulator_cls = dependencies.robot_simulator_cls

        self._meta_file = str(SETTINGS_FILE)
        self._project_dir = str(PROJECT_DIR)
        self._project_store = ProjectConfigStore(self._meta_file, self._project_dir)
        self._project_file = ""
        self._project_name = "default"

        default_project = ProjectConfig()

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
        self._apply_project_config_to_state(default_project)

        self.table_actor = None
        self.robot_sim = self.robot_simulator_cls()
        self.robot_actors = {}

        self._importer_specs = []
        self._postprocessor_specs = []
        self._robot_descriptors = []
        self._import_gcode_path = ""

        self.updating_sliders = False
        self.task_controller = TaskController(self._on_busy_changed, self._set_status_text)

        self.last_ik_solution = None
        self._force_ik_seed_from_config = False
        self._traj_test_started_at = None
        self._traj_test_last_progress = None

        self._render_timer = QTimer()
        self._render_timer.setSingleShot(True)
        self._render_timer.setInterval(60)
        self._render_timer.timeout.connect(self._deferred_update_plot)
        self._pending_tube_render = False

        self._traj_test_feedback_timer = QTimer()
        self._traj_test_feedback_timer.setInterval(1000)
        self._traj_test_feedback_timer.timeout.connect(self._on_traj_test_feedback_timer)

        self._create_main_layout()
        self._create_general_tab()
        self._create_workplace_tab()
        self._create_edit_tab()
        self._create_export_tab()
        self._create_viewport()
        self.trajectory_renderer = TrajectoryRenderer(self.plotter, self.pv)

        self.setup_scene()
