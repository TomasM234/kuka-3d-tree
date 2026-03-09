from pathlib import Path


PACKAGE_NAME = __name__.split(".")[0]
PACKAGE_DIR = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_DIR.parent

APPDATA_DIR = PACKAGE_DIR / "AppData"
CSV_DIR = PACKAGE_DIR / "CSV"
EXTRUDER_DIR = PACKAGE_DIR / "Extruder"
GCODE_DIR = PACKAGE_DIR / "GCODE"
OUTPUT_DIR = PACKAGE_DIR / "Output"
PROJECT_DIR = PACKAGE_DIR / "Project"
SETTINGS_FILE = PACKAGE_DIR / "viewer_settings.json"
SPLASH_PATH = APPDATA_DIR / "Splash.png"


def ensure_directory(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path
