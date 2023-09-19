import os.path

import Src.sim.simhelper as simhelper
import numpy as np
import pybullet as p
import pybullet_planning as pp
from Src.sim.sim_constants import *

from Src.robot.arm.RobotHandler import RobotHandler

import math

# Paths for URDF files.
DATA_DIR = os.path.join(os.path.abspath('../'), "Data", "sim")

URDF_PLANE = os.path.join(DATA_DIR, "urdf", "plane.urdf")
URDF_RBT = os.path.join(DATA_DIR, "urdf", "rx200pantex.urdf")
URDF_PLAT = os.path.join(DATA_DIR, "urdf", "actuated_platform.urdf")
# URDF_CHAMBER = os.path.join(DATA_DIR, "urdf", "chamber.urdf")

class Simulation:
    """
        Simulation class oversees the PyBullet simulation .

        When initialized, a PyBullet client connects to the server, and the robot is simulated.

        Interruptable via. self.can_run
    """

    def __init__(self, time_step=1. / UPDATE_RATE):
        #self.robot_instance = RobotHandler()
        self.goal_conf = []

        # Time management
        self.time_elapsed = 0
        self.time_step = time_step

        # Simulation IDs
        self.sim_id = -1
        self.sim_robot = -1
        self.sim_platform = -1
        self.sim_chamber = -1

        # Simulation ref
        self.current_point_cloud = None

        # Debug stuff
        self.tip_ref_axes = []

        # Initialization stuff
        self.can_run = True
        self.initialize_sim()

    def initialize_sim(self):
        # Instantiate and start a PyBullet physics client
        self.sim_id = pp.connect(use_gui=True)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, PYBULLET_SHOW_GUI, physicsClientId=self.sim_id)
        p.setGravity(0, 0, 0)
        p.setTimeStep(self.time_step)

        # Load ground plane into environment.
        # pp.load_pybullet(URDF_PLANE)

        # Add robot into PyBullet environment
        self.sim_robot = pp.load_pybullet(URDF_RBT, fixed_base=True, scale=SIM_SCALE)
        p.resetBasePositionAndOrientation(self.sim_robot, SIM_ROBOT_OFFSET, p.getQuaternionFromEuler(SIM_ROBOT_ORN))
        print(f"[SIM] Initialized robot with ID: {self.sim_robot}")

        # Add center platform AND OBJECT into PyBullet environment
        self.sim_platform = pp.load_pybullet(URDF_PLAT, fixed_base=True, scale=SIM_SCALE)
        p.resetBasePositionAndOrientation(self.sim_platform, SIM_PLATFORM_OFFSET,
                                          p.getQuaternionFromEuler(SIM_PLATFORM_ORN))
        print(f"[SIM] Initialized platform with ID: {self.sim_platform}")

        # Place chamber in environment
        # self.sim_chamber = pp.load_pybullet(URDF_CHAMBER, fixed_base=True, scale=SIM_SCALE)
        # p.resetBasePositionAndOrientation(self.sim_chamber, SIM_CHAMBER_OFFSET,
        #                                   p.getQuaternionFromEuler(SIM_CHAMBER_ORN))
        # print(f"[SIM] Added chamber model to environment: {self.sim_chamber}")

        # Add scanner arm to environment
        # self.sim_scanner = pp.load_pybullet(URDF_SCANNER, fixed_base=True, scale=SIM_SCALE)
        # p.resetBasePositionAndOrientation(self.sim_scanner, SIM_SCANNER_OFFSET,
        #                                   p.getQuaternionFromEuler(SIM_SCANNER_ORN))
        # print(f"[SIM] Added scanner arm to enivronment: {self.sim_scanner}")

        # Disable chamber collisions
        # p.setCollisionFilterGroupMask(self.sim_chamber, -1, 0, 0)
        # p.setCollisionFilterPair(self.sim_chamber, self.sim_robot, -1, -1, 0)
        # p.setCollisionFilterPair(self.sim_chamber, self.sim_platform, -1, -1, 0)

        pp.set_camera_pose(tuple(np.array((0, 0, 0.25)) + np.array([0.25, -0.25, 0.25])), (0, 0, 0.25))

        if not PYBULLET_SHOW_GUI:
            simhelper.draw_world_axis(1 * SIM_SCALE, 5)

        # create 3 lines with unique IDs for probe tip axis.
        self.tip_ref_axes = [p.addUserDebugLine([0, 0, 0], [0, 0, 0]),
                             p.addUserDebugLine([0, 0, 0], [0, 0, 0]),
                             p.addUserDebugLine([0, 0, 0], [0, 0, 0])]
        if DRAW_TIP_AXES:
            simhelper.draw_tip_axis(self.sim_robot, self.tip_ref_axes)

        #pp.set_joint_positions(self.sim_robot, [2, 3, 4, 5], self.robot_instance.read_cur_conf())
        #self.goal_conf = pp.get_configuration(self.sim_robot)
       #print("current conf: ", self.goal_conf)

        print("[SIM] Successfully initialized PyBullet environment...")

    def update(self, time_elapsed):
        if not self.can_run:
            return

        if DRAW_TIP_AXES:
            simhelper.draw_tip_axis(self.sim_robot, self.tip_ref_axes)

        # pp.control_joints(self.sim_robot, [1,2,3,4,5], pp.inverse_kinematics_helper(self.sim_robot, 6, ([0, 0.15, 0.5],
        #                                               p.getQuaternionFromEuler([0, 0, 0]))))
        #
        # if self.time_elapsed > 5:
        #     self.robot_instance.set_goal_conf(pp.get_configuration(self.sim_robot))

        # Sync with the main instance and therefore real robot.
        self.time_elapsed = time_elapsed

        # Step the simulation
        p.stepSimulation()

    def move_toward_goal(self):
        pass