from dataclasses import dataclass
from importlib.machinery import ModuleSpec
from pathlib import Path
import importlib.util
import sys
import types

from .error_reporting import get_logger


logger = get_logger(__name__)


@dataclass(frozen=True)
class PluginSpec:
    file_name: str
    file_path: str
    module_name: str
    label: str
    module: object

    @property
    def display_name(self):
        return f"{self.label}  [{self.file_name}]" if self.label != self.file_name else self.file_name


def _module_name_from_path(root_dir: str, file_path: str) -> str:
    root = Path(root_dir).resolve()
    path = Path(file_path).resolve()
    rel = path.relative_to(root).with_suffix("")
    return ".".join(rel.parts)


def _ensure_package_context(module_name: str, root_dir: str) -> None:
    package_name = module_name.rpartition(".")[0]
    if not package_name:
        return

    root_path = Path(root_dir).resolve()
    package_parts = package_name.split(".")
    for depth in range(1, len(package_parts) + 1):
        current_name = ".".join(package_parts[:depth])
        current_path = root_path.joinpath(*package_parts[:depth])
        current_path_str = str(current_path)

        existing = sys.modules.get(current_name)
        if existing is not None:
            search_locations = getattr(existing, "__path__", None)
            if search_locations is not None and current_path_str not in search_locations:
                search_locations.append(current_path_str)
            continue

        package_module = types.ModuleType(current_name)
        package_module.__file__ = str(current_path / "__init__.py")
        package_module.__package__ = current_name
        package_module.__path__ = [current_path_str]

        spec = ModuleSpec(current_name, loader=None, is_package=True)
        spec.submodule_search_locations = package_module.__path__
        package_module.__spec__ = spec
        sys.modules[current_name] = package_module


def _load_plugin_module(module_name: str, file_path: Path, root_dir: str):
    _ensure_package_context(module_name, root_dir)
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot create import spec for {file_path}")

    module = importlib.util.module_from_spec(spec)
    previous_module = sys.modules.get(module_name)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        if previous_module is None:
            sys.modules.pop(module_name, None)
        else:
            sys.modules[module_name] = previous_module
        raise
    return module


def discover_python_plugins(directory: str, root_dir: str | None = None, required_attrs: tuple[str, ...] = ()):
    root_dir = root_dir or directory
    specs = []
    base = Path(directory)
    if not base.is_dir():
        return specs

    root_dir = str(Path(root_dir).resolve())
    for path in sorted(base.glob("*.py")):
        if path.name == "__init__.py" or path.stat().st_size == 0:
            continue
        module_name = _module_name_from_path(root_dir, str(path))
        try:
            mod = _load_plugin_module(module_name, path, root_dir)
            if getattr(mod, "HIDE_FROM_UI", False):
                continue
            if any(not hasattr(mod, attr) for attr in required_attrs):
                continue
            label = mod.get_label() if hasattr(mod, "get_label") else path.name
            specs.append(
                PluginSpec(
                    file_name=path.name,
                    file_path=str(path),
                    module_name=module_name,
                    label=label,
                    module=mod,
                )
            )
        except Exception:
            logger.exception("Failed to load plugin %s", path.name)

    return specs
