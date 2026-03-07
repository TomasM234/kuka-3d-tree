from dataclasses import dataclass
from pathlib import Path
import importlib.util


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


def discover_python_plugins(directory: str, root_dir: str | None = None, required_attrs: tuple[str, ...] = ()):
    root_dir = root_dir or directory
    specs = []
    base = Path(directory)
    if not base.is_dir():
        return specs

    for path in sorted(base.glob("*.py")):
        if path.name == "__init__.py" or path.stat().st_size == 0:
            continue
        try:
            spec = importlib.util.spec_from_file_location(path.stem, path)
            if spec is None or spec.loader is None:
                raise ImportError("module spec has no loader")
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            if getattr(mod, "HIDE_FROM_UI", False):
                continue
            if any(not hasattr(mod, attr) for attr in required_attrs):
                continue
            label = mod.get_label() if hasattr(mod, "get_label") else path.name
            specs.append(
                PluginSpec(
                    file_name=path.name,
                    file_path=str(path),
                    module_name=_module_name_from_path(root_dir, str(path)),
                    label=label,
                    module=mod,
                )
            )
        except Exception as exc:
            print(f"[Plugin] Failed to load {path.name}: {exc}")

    return specs
