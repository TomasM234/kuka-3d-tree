import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from PyQt6.QtCore import QThread, pyqtSignal

from .error_reporting import get_logger


logger = get_logger(__name__)


@dataclass(frozen=True)
class RobotDescriptor:
    name: str
    file_path: str
    label: str


def robot_name_from_urdf(file_path: str) -> str:
    fallback = os.path.splitext(os.path.basename(file_path))[0]
    try:
        root = ET.parse(file_path).getroot()
        name_attr = root.get("name")
        if name_attr:
            return name_attr
    except Exception:
        logger.exception("Failed to parse URDF name from %s", file_path)
    return fallback


def discover_robot_descriptors(root_dir: str):
    robots = []
    for entry in sorted(os.listdir(root_dir)):
        sub = os.path.join(root_dir, entry)
        if not os.path.isdir(sub):
            continue
        urdf_dir = os.path.join(sub, "urdf")
        if not os.path.isdir(urdf_dir):
            continue
        for fname in sorted(os.listdir(urdf_dir)):
            if not fname.endswith(".urdf"):
                continue
            fpath = os.path.join(urdf_dir, fname)
            robots.append((robot_name_from_urdf(fpath), fpath))

    if not robots:
        return []

    name_counts = {}
    for robot_name, _ in robots:
        name_counts[robot_name] = name_counts.get(robot_name, 0) + 1

    descriptors = []
    for robot_name, fpath in robots:
        if name_counts[robot_name] > 1:
            rel_path = os.path.relpath(fpath, root_dir)
            label = f"{robot_name} [{rel_path}]"
        else:
            label = robot_name
        descriptors.append(RobotDescriptor(name=robot_name, file_path=fpath, label=label))
    return descriptors


class RobotLoadThread(QThread):
    """Background URDF loader that builds a RobotSimulator off the GUI thread."""

    finished_signal = pyqtSignal(bool, object, str, str)

    def __init__(self, simulator_cls, file_path: str):
        super().__init__()
        self.simulator_cls = simulator_cls
        self.file_path = file_path

    def run(self):
        try:
            simulator = self.simulator_cls()
            simulator.load_robot(self.file_path)
            self.finished_signal.emit(True, simulator, robot_name_from_urdf(self.file_path), "")
        except Exception as exc:
            logger.exception("Robot load failed for %s", self.file_path)
            self.finished_signal.emit(False, None, "", str(exc))
