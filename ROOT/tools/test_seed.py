import os

import numpy as np

from ..app_paths import PACKAGE_DIR
from ..error_reporting import get_logger
from ..robot_ik import RobotSimulator


logger = get_logger(__name__)


def main():
    sim = RobotSimulator()
    urdf_file = os.path.join(PACKAGE_DIR, "kuka_robot_descriptions-master", "urdf", "kr210_r2700_2.urdf")
    sim.load_robot(urdf_file)

    logger.info("Active joints: %s", sim.active_joints)
    logger.info("IK links:")
    for index, link in enumerate(sim.ik_chain.links):
        logger.info("  %s: name=%s", index, link.name)

    angles = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    seed = np.zeros(len(sim.ik_chain.links))
    angle_idx = 0
    for index, link in enumerate(sim.ik_chain.links):
        if link.name in sim.active_joints and angle_idx < len(angles):
            seed[index] = angles[angle_idx]
            logger.info("Mapped angle %s to index %s (link %s)", angles[angle_idx], index, link.name)
            angle_idx += 1

    logger.info("Resulting SEED: %s", seed)
    logger.info("Does ikpy use link.name equal to active_joints?")


if __name__ == "__main__":
    main()
