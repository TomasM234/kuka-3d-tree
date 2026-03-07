import json
import os
from dataclasses import dataclass


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

        return cls(
            print_thickness=_to_float(payload.get("print_thickness"), default.print_thickness),
            last_file_path=str(payload.get("last_file_path", default.last_file_path) or ""),
            last_postprocessor=str(payload.get("last_postprocessor", default.last_postprocessor) or ""),
            last_urdf_path=str(payload.get("last_urdf_path", default.last_urdf_path) or ""),
            base_x=_to_float(payload.get("base_x"), default.base_x),
            base_y=_to_float(payload.get("base_y"), default.base_y),
            base_z=_to_float(payload.get("base_z"), default.base_z),
            base_a=_to_float(payload.get("base_a"), default.base_a),
            base_b=_to_float(payload.get("base_b"), default.base_b),
            base_c=_to_float(payload.get("base_c"), default.base_c),
            tool_x=_to_float(payload.get("tool_x"), default.tool_x),
            tool_y=_to_float(payload.get("tool_y"), default.tool_y),
            tool_z=_to_float(payload.get("tool_z"), default.tool_z),
            tool_a=_to_float(payload.get("tool_a"), default.tool_a),
            tool_b=_to_float(payload.get("tool_b"), default.tool_b),
            tool_c=_to_float(payload.get("tool_c"), default.tool_c),
            table_x1=_to_float(payload.get("table_x1"), default.table_x1),
            table_y1=_to_float(payload.get("table_y1"), default.table_y1),
            table_x2=_to_float(payload.get("table_x2"), default.table_x2),
            table_y2=_to_float(payload.get("table_y2"), default.table_y2),
            show_table=_to_bool(payload.get("show_table"), default.show_table),
            show_robot=_to_bool(payload.get("show_robot"), default.show_robot),
            ik_config=str(payload.get("ik_config", default.ik_config) or default.ik_config)
        )

    def to_dict(self):
        """Return a JSON-serializable dictionary."""
        return {
            "print_thickness": self.print_thickness,
            "last_file_path": self.last_file_path,
            "last_postprocessor": self.last_postprocessor,
            "last_urdf_path": self.last_urdf_path,
            "base_x": self.base_x,
            "base_y": self.base_y,
            "base_z": self.base_z,
            "base_a": self.base_a,
            "base_b": self.base_b,
            "base_c": self.base_c,
            "tool_x": self.tool_x,
            "tool_y": self.tool_y,
            "tool_z": self.tool_z,
            "tool_a": self.tool_a,
            "tool_b": self.tool_b,
            "tool_c": self.tool_c,
            "table_x1": self.table_x1,
            "table_y1": self.table_y1,
            "table_x2": self.table_x2,
            "table_y2": self.table_y2,
            "show_table": self.show_table,
            "show_robot": self.show_robot,
            "ik_config": self.ik_config
        }


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

    def load_last_project_path(self):
        filename = self.default_project
        if os.path.exists(self.meta_file):
            try:
                with open(self.meta_file, "r", encoding="utf-8") as f:
                    meta = json.load(f)
                if isinstance(meta, dict):
                    candidate = meta.get("last_project")
                    if isinstance(candidate, str) and candidate.strip():
                        filename = self.normalize_project_filename(candidate)
            except Exception:
                filename = self.default_project
        return self.project_path(filename)

    def save_last_project_path(self, project_path):
        try:
            filename = self.normalize_project_filename(os.path.basename(project_path))
        except ValueError:
            filename = self.default_project
        with open(self.meta_file, "w", encoding="utf-8") as f:
            json.dump({"last_project": filename}, f, indent=4)

    @staticmethod
    def load_project(project_path):
        if not os.path.exists(project_path):
            return ProjectConfig()
        try:
            with open(project_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return ProjectConfig.from_dict(data)
        except Exception:
            return ProjectConfig()

    @staticmethod
    def save_project(project_path, config):
        os.makedirs(os.path.dirname(project_path), exist_ok=True)
        with open(project_path, "w", encoding="utf-8") as f:
            json.dump(config.to_dict(), f, indent=4)
