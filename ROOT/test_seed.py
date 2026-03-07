import sys
import os
import numpy as np

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from robot_ik import RobotSimulator

sim = RobotSimulator()
urdf_file = os.path.join(ROOT_DIR, 'kuka_robot_descriptions-master', 'urdf', 'kr210_r2700_2.urdf')
sim.load_robot(urdf_file)

print("Active joints:", sim.active_joints)
print("IK links:")
for i, l in enumerate(sim.ik_chain.links):
    print(f"  {i}: name={l.name}")

angles = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]  # dummy unmistakable values
seed = np.zeros(len(sim.ik_chain.links))
angle_idx = 0
for i, link in enumerate(sim.ik_chain.links):
    if link.name in sim.active_joints and angle_idx < len(angles):
        seed[i] = angles[angle_idx]
        print(f"Mapped angle {angles[angle_idx]} to index {i} (link {link.name})")
        angle_idx += 1

print("\nResulting SEED:", seed)
print("Does ikpy use link.name equal to active_joints?")
