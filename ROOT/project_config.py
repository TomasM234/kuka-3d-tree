import json
import os
from dataclasses import asdict, dataclass, fields

from .error_reporting import get_logger


logger = get_logger(__name__)


DEFAULT_PROJECT_FILENAME = "default.json"
DEFAULT_IK_CONFIG = "Elbow Up / Front"


def _to_float(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _to_bool(value, default):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    return default


@dataclass
class ProjectConfig:
    """Persisted per-project viewer settings."""

    print_thickness: float = 1.0
    last_file_path: str = ""
    last_postprocessor: str = ""
    last_urdf_path: str = ""
    base_x: float = 0.0
    base_y: float = 0.0
    base_z: float = 0.0
    base_a: float = 0.0
    base_b: float = 0.0
    base_c: float = 0.0
    tool_x: float = 0.0
    tool_y: float = 0.0
    tool_z: float = 0.0
    tool_a: float = 0.0
    tool_b: float = 0.0
    tool_c: float = 0.0
    table_x1: float = 0.0
    table_y1: float = 0.0
    table_x2: float = 550.0
    table_y2: float = 650.0
    show_table: bool = True
    show_robot: bool = True
    ik_config: str = DEFAULT_IK_CONFIG

    @classmethod
    def from_dict(cls, payload):
        """Create config from a raw JSON dictionary."""
        default = cls()
        if not isinstance(payload, dict):
            return default

        values = {}
        for field in fields(cls):
            default_value = getattr(default, field.name)
            raw_value = payload.get(field.name, default_value)
            if isinstance(default_value, bool):
                values[field.name] = _to_bool(raw_value, default_value)
            elif isinstance(default_value, float):
                values[field.name] = _to_float(raw_value, default_value)
            elif isinstance(default_value, str):
                values[field.name] = str(raw_value or default_value)
            else:
                values[field.name] = raw_value

        return cls(**values)

    def to_dict(self):
        """Return a JSON-serializable dictionary."""
        return asdict(self)


PROJECT_STATE_ATTRS = tuple(field.name for field in fields(ProjectConfig))


class ProjectConfigStore:
    """File-system backed project/config storage."""

    def __init__(self, meta_file, project_dir, default_project=DEFAULT_PROJECT_FILENAME):
        self.meta_file = meta_file
        self.project_dir = project_dir
        os.makedirs(self.project_dir, exist_ok=True)
        self.default_project = self.normalize_project_filename(default_project)

    @staticmethod
    def normalize_project_filename(name):
        clean = str(name or "").strip()
        if not clean:
            raise ValueError("Project name cannot be empty.")
        if not clean.lower().endswith(".json"):
            clean += ".json"
        return os.path.basename(clean)

    def project_path(self, name_or_filename):
        filename = self.normalize_project_filename(name_or_filename)
        return os.path.join(self.project_dir, filename)

    @staticmethod
    def project_name_from_path(path):
        return os.path.splitext(os.path.basename(path))[0]

    def list_projects(self):
        try:
            files = sorted(
                name for name in os.listdir(self.project_dir)
                if name.lower().endswith(".json")
            )
        except OSError:
            return []
        return [self.project_path(name) for name in files]

    def _read_meta_payload(self):
        if not os.path.exists(self.meta_file):
            return {}
        try:
            with open(self.meta_file, "r", encoding="utf-8") as f:
                payload = json.load(f)
            if isinstance(payload, dict):
                return payload
        except Exception:
            logger.exception("Failed to load viewer metadata from %s", self.meta_file)
        return {}

    def _write_meta_payload(self, payload):
        with open(self.meta_file, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=4)

    def load_last_project_path(self):
        filename = self.default_project
        meta = self._read_meta_payload()
        candidate = meta.get("last_project")
        if isinstance(candidate, str) and candidate.strip():
            try:
                filename = self.normalize_project_filename(candidate)
            except ValueError:
                filename = self.default_project
        return self.project_path(filename)

    def save_last_project_path(self, project_path):
        payload = self._read_meta_payload()
        try:
            filename = self.normalize_project_filename(os.path.basename(project_path))
        except ValueError:
            filename = self.default_project
        payload["last_project"] = filename
        self._write_meta_payload(payload)

    def load_dock_layout_state(self):
        payload = self._read_meta_payload()
        dock_state = payload.get("dock_layout_state", "")
        return dock_state if isinstance(dock_state, str) else ""

    def save_dock_layout_state(self, dock_state):
        payload = self._read_meta_payload()
        if isinstance(dock_state, str) and dock_state:
            payload["dock_layout_state"] = dock_state
        else:
            payload.pop("dock_layout_state", None)
        self._write_meta_payload(payload)

    @staticmethod
    def load_project(project_path):
        if not os.path.exists(project_path):
            return ProjectConfig()
        try:
            with open(project_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return ProjectConfig.from_dict(data)
        except Exception:
            logger.exception("Failed to load project config from %s", project_path)
            return ProjectConfig()

    @staticmethod
    def save_project(project_path, config):
        os.makedirs(os.path.dirname(project_path), exist_ok=True)
        with open(project_path, "w", encoding="utf-8") as f:
            json.dump(config.to_dict(), f, indent=4)
