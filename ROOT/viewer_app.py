import argparse
from dataclasses import dataclass
import multiprocessing
import sys

from PyQt6.QtCore import QElapsedTimer, QTimer, Qt
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import QApplication, QSplashScreen

from .app_paths import SPLASH_PATH
from .error_reporting import configure_logging, get_logger
from .viewer import RobotPathViewer


logger = get_logger(__name__)


@dataclass(frozen=True)
class ViewerDependencies:
    pv_module: object
    qt_interactor_cls: object
    robot_simulator_cls: object


def load_viewer_dependencies() -> ViewerDependencies:
    import pyvista
    from pyvistaqt import QtInteractor

    from .robot_ik import RobotSimulator

    return ViewerDependencies(
        pv_module=pyvista,
        qt_interactor_cls=QtInteractor,
        robot_simulator_cls=RobotSimulator,
    )


def create_splash(app: QApplication):
    if not SPLASH_PATH.exists():
        return None

    pixmap = QPixmap(str(SPLASH_PATH))
    screen = app.primaryScreen().geometry()
    max_width = int(screen.width() * 0.8)
    max_height = int(screen.height() * 0.8)
    if pixmap.width() > max_width or pixmap.height() > max_height:
        pixmap = pixmap.scaled(
            max_width,
            max_height,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )

    splash = QSplashScreen(pixmap, Qt.WindowType.WindowStaysOnTopHint | Qt.WindowType.SplashScreen)
    splash.show()
    splash.raise_()
    app.processEvents()
    return splash


def build_window(dependencies: ViewerDependencies) -> RobotPathViewer:
    return RobotPathViewer(dependencies)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Launch the ROOT trajectory viewer.")
    parser.add_argument("--no-splash", action="store_true", help="Start without the splash screen.")
    return parser.parse_args(argv)


def main(argv=None):
    configure_logging()
    multiprocessing.freeze_support()
    args = parse_args(argv)

    app = QApplication([sys.argv[0]])
    splash = None if args.no_splash else create_splash(app)

    try:
        dependencies = load_viewer_dependencies()
        app.processEvents()
        load_timer = QElapsedTimer()
        load_timer.start()

        window = build_window(dependencies)
        window.show()

        if splash is not None:
            remaining_ms = max(0, 3000 - load_timer.elapsed())
            QTimer.singleShot(remaining_ms, lambda: splash.finish(window))

        return app.exec()
    except Exception:
        logger.exception("Viewer startup failed")
        raise


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
