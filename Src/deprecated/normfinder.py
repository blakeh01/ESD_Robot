import os.path

import pybullet as p
import pybullet_data
import time

from dynio import *

from src.sim.sim_constants import *

def chunk(array, size_limit):
    out = []
    l = len(array)
    s = 0

    while l - s > 0:
        out.append(array[s:s+size_limit])
        s += size_limit

    return out

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

# directory management
HERE = os.path.abspath('..')
RBT_PATH = os.path.join(HERE, 'data', 'universal_robot', 'ur_description', 'urdf', 'ur5.urdf')



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


## END SETUP


# Define some ray starting and end points depending on variables.
center = [0, 0, 0]
z_lower_limit = 0.25
z_upper_limit = 0.5
xz_offset = 0.25
resolution = 360  # ex. 1 raybatch per degree
z_density = 250

z_space = np.linspace(z_lower_limit, z_upper_limit, z_density, endpoint=True)
theta_space = np.arange(0, 2*np.pi, (2*np.pi)/resolution)

ray_to_positions = []
ray_from_positions = []

for z in z_space:
    for theta in theta_space:
        x_coord = xz_offset * np.cos(theta)
        y_coord = xz_offset * np.sin(theta)

        ray_from_positions.append([x_coord+center[0], y_coord+center[1], z])
        ray_to_positions.append([center[0], center[1], z])


# Batch our rays to avoid ray limit
batched_froms = chunk(ray_from_positions, p.MAX_RAY_INTERSECTION_BATCH_SIZE)
batched_tos = chunk(ray_to_positions, p.MAX_RAY_INTERSECTION_BATCH_SIZE)
points = []

for i in range(len(batched_froms)):
    froms = batched_froms[i]
    tos = batched_tos[i]

    # Shoot ray to measure normals.
    _output = p.rayTestBatch(froms, tos,
                             numThreads=0)  # setting numthreads=0 makes use of ALL available threads.

    length_of_norm = 0.05
    dist_from_obj = 0.005
    draw_norms = False

    for result in _output:
        if not result[1] == -1:
            hit_pos = result[3]  # also the 'start' point of the normal vector
            hit_norm = result[4]
            end_point = np.add(np.dot(hit_norm, length_of_norm), hit_pos)

            if draw_norms: p.addUserDebugLine(hit_pos, end_point, [0, 1, 1], 1)

            pos = np.add(hit_pos, np.dot(hit_norm, dist_from_obj))
            points.append(pos)


colors = []
for i in range(len(points)):
    colors.append([0, 0, 1])

p.addUserDebugPoints(points, colors, 2)

# Run sim loop.

while(1):
    p.stepSimulation()
    time.sleep(dt)

