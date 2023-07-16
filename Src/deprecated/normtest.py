import pybullet as p
import time

import pybullet_data

from src.sim.sim_constants import *

from pybullet_planning import inverse_kinematics_helper


# Testcase Takeaways:
#   For complex objects of high polygon density, you MUST use the pybullet VHACD translator tool
#   to get proper collision boxes.


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def move(t, pos, rot):
    poses = inverse_kinematics_helper(sim_robot_id, 6,
                                      (pos, p.getQuaternionFromEuler([0, rot, -np.pi / 2])))

    # robot_instance.write_joint_states(poses)
    p.setJointMotorControlArray(sim_robot_id, [0, 2, 3, 4, 5],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=poses,
                                forces=[1000, 1000, 1000, 1000, 1000],
                                positionGains=[1, 1, 1, 1, 1],
                                velocityGains=[1, 1, 1, 1, 1])


def chk_col():
    dist_list = []
    for pt in p.getClosestPoints(sim_robot_id, sim_platform_id, 0.2):
        dist_list.append(pt[8])
    if len(dist_list) > 0:
        print(min(dist_list))


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Add robot into PyBullet environment
sim_robot_pos = np.dot(2, SIM_ROBOT_OFFSET)
sim_robot_orn = p.getQuaternionFromEuler(SIM_ROBOT_ORN)
sim_robot_id = p.loadURDF("pantex/urdf/rx200pantex_updated.urdf", sim_robot_pos, sim_robot_orn, useFixedBase=True,
                          globalScaling=2)
p.resetBasePositionAndOrientation(sim_robot_id, sim_robot_pos, sim_robot_orn)
print(f"Initialized robot with ID: {sim_robot_id}")

# Add platform
sim_platform_id = p.loadURDF("pantex/urdf/rot_base.urdf", SIM_PLATFORM_OFFSET, p.getQuaternionFromEuler([0, 0, 0]),
                             globalScaling=2)
p.resetBasePositionAndOrientation(sim_platform_id, SIM_PLATFORM_OFFSET, p.getQuaternionFromEuler([0, 0, 0]))
print(f"Initialized platform with ID: {sim_platform_id}")

dt = 1. / 60.
p.setTimeStep(dt)

p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 6, 0, 0)
p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 5, 0, 0)
p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 4, 0, 0)
p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 6, 1, 0)
p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 5, 1, 0)
p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 4, 1, 0)
p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 6, -1, 0)
p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 5, -1, 0)
p.setCollisionFilterPair(sim_robot_id, sim_platform_id, 4, -1, 0)


def get_sim_joint_states():
    return [p.getJointState(sim_robot_id, 0)[0], p.getJointState(sim_robot_id, 2)[0],
            p.getJointState(sim_robot_id, 3)[0], p.getJointState(sim_robot_id, 4)[0],
            p.getJointState(sim_robot_id, 5)[0]]


## PROBING ALGOR ... proper implementation: rotate around the center of the platform shooting rays every degree or so.
bound_a = [-0.1, 0, 0.25]
bound_b = [0.1, 0, 0.5]

ray_start_offset = [0, 1, 0]
ray_end_offset = [0, 0, 0]
ray_density_x = 1
ray_density_z = 100

ray_from_positions = []
ray_to_positions = []

interval_x = np.linspace(bound_a[0], bound_b[0], ray_density_x)
interval_z = np.linspace(bound_a[2], bound_b[2], ray_density_z)

for x in interval_x:
    for z in interval_z:
        ray_from_positions.append([0, 0.3, z])
        ray_to_positions.append([0, 0, z])

# Draw the rays
for i in range(len(ray_from_positions)):
    break
    p.addUserDebugLine(ray_from_positions[i], ray_to_positions[i], [1, 0, 0], 2)

p.stepSimulation()
time.sleep(dt)

# Shoot out rays towards object
_output = p.rayTestBatch(ray_from_positions, ray_to_positions)

p.stepSimulation()
time.sleep(dt)

normals_p0 = []
normals_dir = []
normals_theta = []

length_of_norm = 0.25

# Iterate through each of the rays and store their hit positions and hit normals
# Also draw a line in the direction of the normal.
# Also, find the angle the normal makes with relation to the horizontal plane.
for result in _output:
    if not result[1] == -1:
        hit_pos = result[3]  # also the 'start' point of the normal vector
        hit_norm = result[4]
        end_point = np.add(np.dot(hit_norm, length_of_norm), hit_pos)

        p.addUserDebugLine(hit_pos, end_point, [0, 1, 1], 1)

        # WARNING: uses hard coded ground plane angles (will only work on singular axis)
        normals_theta.append(
            np.arccos(np.clip(np.dot(unit_vector(hit_norm), unit_vector(np.array([0, 1, 0]))), -1.0, 1.0)))
        normals_p0.append(hit_pos)
        normals_dir.append(hit_norm)

p.stepSimulation()
time.sleep(dt)

# TURN ON ROBOT


# print("WARNING: MATCH JOINT POSITIONS AS YOU SEE..")
# wait_for_user()
# robot_instance = RobotHandler()

while (1):
    p.stepSimulation()
    time.sleep(dt)

    for i in range(20, len(normals_p0) - 5):
        move(0, alignment_points[i], probe_angle[i])
        p.stepSimulation()
        chk_col()
        time.sleep(dt)

    for i in range(len(normals_p0) - 6, 20, -1):
        move(0, alignment_points[i], probe_angle[i])
        p.stepSimulation()
        chk_col()
        time.sleep(dt)
for i in range(0, 5):
    set_joint_positions(sim_robot_id, get_movable_joints(sim_robot_id),
                        inverse_kinematics_helper(sim_robot_id, 6,
                                                  (alignment_points[50],
                                                   p.getQuaternionFromEuler([0, probe_angle[50], -np.pi / 2]))))
