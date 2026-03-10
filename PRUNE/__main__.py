if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parent.parent
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    __package__ = "PRUNE"

import argparse
import logging
import multiprocessing
import sys
from pathlib import Path

from PyQt6.QtCore import QElapsedTimer, QTimer, Qt
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import QApplication, QSplashScreen


logger = logging.getLogger(__name__)

SPLASH_PATH = Path(__file__).resolve().parent / "AppData" / "SplashPrune.png"


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


def configure_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Launch the PRUNE CAD processor.")
    parser.add_argument("--no-splash", action="store_true", help="Start without the splash screen.")
    return parser.parse_args(argv)

def main(argv=None):
    configure_logging()
    multiprocessing.freeze_support()
    args = parse_args(argv)

    app = QApplication([sys.argv[0]])
    splash = None if args.no_splash else create_splash(app)

    try:
        app.processEvents()
        load_timer = QElapsedTimer()
        load_timer.start()

        from PRUNE.ui_main import PruneMainWindow
        window = PruneMainWindow()
        window.show()

        if splash is not None:
            remaining_ms = max(0, 3000 - load_timer.elapsed())
            QTimer.singleShot(remaining_ms, lambda: splash.finish(window))

        return app.exec()
    except Exception:
        logger.exception("PRUNE startup failed")
        raise

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
