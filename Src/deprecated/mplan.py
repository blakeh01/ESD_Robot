# testcase class for motion planning
import os
import time

import numpy as np
import pybullet as p
import pybullet_planning as pp
from src.sim.sim_constants import SIM_ROBOT_OFFSET, SIM_ROBOT_ORN, SIM_SCALE

DATA_DIR = os.path.join(os.path.abspath('../..'), "data", "sim")
URDF_RBT = os.path.join(DATA_DIR, "urdf", "rx200pantex.urdf")

pp.connect(use_gui=True)

robot = pp.load_pybullet(URDF_RBT, scale=SIM_SCALE)
p.resetBasePositionAndOrientation(robot, SIM_ROBOT_OFFSET, p.getQuaternionFromEuler(SIM_ROBOT_ORN))

rbt_joints = [0, 2, 3, 4, 5]

p.addUserDebugPoints([[0.1, 0, 0.5]], [[1, 0, 0]], 5)

pp.set_camera_pose(tuple(np.array((0, 0.1, 0)) + np.array([0.25, -0.25, 0.25])), (0, 0.1, 0))


def create_path_plan():
    startPos = pp.get_joint_positions(robot, rbt_joints)

    for i in range(1, 10):
        endPos = p.calculateInverseKinematics(robot, 6, [0.1, 0, 0.5],
                                              p.getQuaternionFromEuler([0, 0, 0]))

    path_to = pp.plan_joint_motion(robot, rbt_joints, endPos)

    for _ in path_to:
        print(_)

    pp.set_joint_positions(robot, rbt_joints, startPos)

    traj = pp.trajectory_controller(robot, rbt_joints, path_to, tolerance=0.05)
    s_gen = pp.simulate_controller(traj)

    return s_gen


input("a")

s_gen = create_path_plan()

while 1:

    try:
        next(s_gen)
    except:
        input("at finished pos, try again?")
        s_gen = create_path_plan()

    p.stepSimulation()
    time.sleep(1. / 60)

p.disconnect()
