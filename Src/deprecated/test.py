import os
import time
import logging

import pybullet as p
import pybullet_planning as pp
#import pybulletX as pX

from src.sim.sim_constants import *
from src.sim.simhelper import get_normal_point_cloud


# temp class for refactoring code, also notetaking revert

# for probing algorithm, match normals of surface to probe normal...?

def focus_camera(focus_point, offset):
    camera_pt = np.array(focus_point) + np.array(offset)
    pp.set_camera_pose(tuple(camera_pt), focus_point)


# logging example, can be used for event handling perhaps?

logger = logging.getLogger('simulation')
logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# add formatter to ch
ch.setFormatter(formatter)

# add ch to logger
logger.addHandler(ch)


## define some paths, first path can be put in somekind of configuration perhaps?

DATA_DIR = os.path.join(os.path.abspath('../..'), "data", "sim")

URDF_PLANE = os.path.join(DATA_DIR, "urdf", "plane.urdf")
URDF_RBT = os.path.join(DATA_DIR, "urdf", "rx200pantex_updated.urdf")
URDF_PLAT = os.path.join(DATA_DIR, "urdf", "rot_base.urdf")
URDF_CHAMBER = os.path.join(DATA_DIR, "urdf", "chamber.urdf")

SIM_SCALE = 2  # every 1 meter in real life is 2 meters in sim.


## connect
sim_id = pp.connect(use_gui=True)

# enable/disable GUI
p.configureDebugVisualizer(p.COV_ENABLE_GUI, False, physicsClientId=sim_id)

# load urdfs into enviroment and set their pos
chamber = pp.load_pybullet(URDF_CHAMBER, fixed_base=True, scale=SIM_SCALE)
p.resetBasePositionAndOrientation(chamber, np.dot([0, 0.2, 0], 2), p.getQuaternionFromEuler([0, 0, 0]))
sim_robot = pp.load_pybullet(URDF_RBT, fixed_base=True, scale=SIM_SCALE)
p.resetBasePositionAndOrientation(sim_robot, np.dot(SIM_ROBOT_OFFSET, 2), p.getQuaternionFromEuler(SIM_ROBOT_ORN))
sim_plat = pp.load_pybullet(URDF_PLAT, fixed_base=True, scale=SIM_SCALE)
p.resetBasePositionAndOrientation(sim_plat, SIM_PLATFORM_OFFSET, p.getQuaternionFromEuler([0, 0, 0]))

# Disable chamber collisions
p.setCollisionFilterGroupMask(chamber, -1, 0, 0)
p.setCollisionFilterPair(chamber, sim_robot, -1, -1, 0)
p.setCollisionFilterPair(chamber, sim_plat, -1, -1, 0)

logger.info("Loaded environmental objects!")

focus_camera([0, 0, 0], [1, 0, 0.5])

refX = p.addUserDebugLine([0, 0, 0], [0, 0, 0])
refY = p.addUserDebugLine([0, 0, 0], [0, 0, 0])
refZ = p.addUserDebugLine([0, 0, 0], [0, 0, 0])

# draw joint names
#for i in range(p.getNumJoints(sim_robot)):
#    pp.draw_link_name(sim_robot, i)

vals = get_normal_point_cloud()

while(1):

    p.stepSimulation()
    time.sleep(1./120)