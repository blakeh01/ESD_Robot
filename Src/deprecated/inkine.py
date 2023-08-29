import time

import pybullet as p
import pybullet_data
from pybullet_planning import set_camera_pose, inverse_kinematics_helper, \
    get_movable_joints, set_joint_positions, wait_for_user
from src.robot.RobotHandler import RobotHandler
from src.sim.sim_constants import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

dt = 1. / 60.
p.setTimeStep(dt)

# Add robot into PyBullet environment
sim_robot_pos = SIM_ROBOT_OFFSET
sim_robot_orn = p.getQuaternionFromEuler(SIM_ROBOT_ORN)
sim_robot_id = p.loadURDF("pantex/urdf/rx200pantex_updated.urdf", sim_robot_pos, sim_robot_orn, useFixedBase=True,
                          globalScaling=1)
p.resetBasePositionAndOrientation(sim_robot_id, sim_robot_pos, sim_robot_orn)
print(f"Initialized robot with ID: {sim_robot_id}")

set_camera_pose(tuple(np.array((0, 0.1, 0)) + np.array([0.25, -0.25, 0.25])), (0, 0.1, 0))

# Add center platform AND OBJECT into PyBullet environment
# sim_platform_id = p.loadURDF("pantex/urdf/rot_base.urdf", SIM_PLATFORM_OFFSET, p.getQuaternionFromEuler([0,0,0]))
# p.resetBasePositionAndOrientation(sim_platform_id, SIM_PLATFORM_OFFSET, p.getQuaternionFromEuler([0,0,0]))
# print(f"Initialized platform with ID: {sim_platform_id}")

#############

param_x = p.addUserDebugParameter("X", -1, 1, 0)
param_y = p.addUserDebugParameter("Y", -1, 1, 0)
param_z = p.addUserDebugParameter("Z", -1, 1, 0)

param_rx = p.addUserDebugParameter("RX", -90, 90, 0)

param_pt = p.addUserDebugParameter("Show Point", 1, 0, 1)
hasDisp = False

param_btn = p.addUserDebugParameter("Go", 1, 0, 1)
move_flag = False

bt = p.addUserDebugPoints([[0, 0, 0]], [[1, 0, 0]], 5)

movement_tolerance = 0.005
curpose = None

y_off = p.addUserDebugParameter("Y", -1, 1, 0.1475)
z_off = p.addUserDebugParameter("Z", -0.1, 0.1, -0.0165)


#############
def move(t, pos, orn):
    rot = 30 * np.cos(t)
    poses = inverse_kinematics_helper(sim_robot_id, 6,
                                      ([0, 0.05, 0.3], p.getQuaternionFromEuler([0, rot * (np.pi / 180), -np.pi / 2])))

    robot_instance.write_joint_states(poses)
    p.setJointMotorControlArray(sim_robot_id, [0, 2, 3, 4, 5],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=poses,
                                forces=[1000, 1000, 1000, 1000, 1000],
                                positionGains=[1, 1, 1, 1, 1],
                                velocityGains=[1, 1, 1, 1, 1])


def get_sim_joint_states():
    return [p.getJointState(sim_robot_id, 0)[0], p.getJointState(sim_robot_id, 2)[0],
            p.getJointState(sim_robot_id, 3)[0], p.getJointState(sim_robot_id, 4)[0],
            p.getJointState(sim_robot_id, 5)[0]]


for i in range(0, 5):
    set_joint_positions(sim_robot_id, get_movable_joints(sim_robot_id),
                        inverse_kinematics_helper(sim_robot_id, 6,
                                                  ([0, 0.05, 0.3], p.getQuaternionFromEuler([0, 0, -np.pi / 2]))))

print("WARNING: MATCH JOINT POSITIONS AS YOU SEE..")
wait_for_user()
robot_instance = RobotHandler()

t = 0
while (1):
    t += dt
    x = p.readUserDebugParameter(param_x)
    y = p.readUserDebugParameter(param_y)
    z = p.readUserDebugParameter(param_z)

    p.addUserDebugPoints([[x, y, z]], [[1, 0, 0]], 5, lifeTime=0.01)

    if p.readUserDebugParameter(param_btn) % 2 == 0:
        move_flag = True

    if move_flag:
        curpose = move(t, [x, y, z], p.getQuaternionFromEuler([0, 0, 0]))

    p.stepSimulation()
    time.sleep(dt)
