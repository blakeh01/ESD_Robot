import os.path

import Src.sim.simhelper as simhelper
import numpy as np
import pybullet as p
import pybullet_planning as pp
from Src.sim.sim_constants import *
from Src.sim.Command import *

from Src.robot.arm.RobotHandler import RobotHandler
from Src.util.math_util import *

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

    def __init__(self, parent, time_step=1. / UPDATE_RATE):
        self.parent = parent
        self.robot_handler = RobotHandler()

        self.pos_probe_command = None
        self.pos_plat_command = None

        # Time management
        self.time_elapsed = 0
        self.time_step = time_step

        # Simulation IDs
        self.sim_id = -1
        self.sim_robot = -1
        self.sim_platform = -1
        self.sim_chamber = -1

        # Simulation ref
        self.normal_point_cloud = None
        self.cur_probe_flow = None
        self.can_execute_flow = False
        self.lineup_normal = [-1, 0, 0] # direction of the normal vector that 'lines up' the platform to the robot.

        # Debug stuff
        self.tip_ref_axes = []

        # Initialization stuff
        self.can_run = True
        self.initialize_sim()

        p.addUserDebugLine([0, 0, 0], self.lineup_normal, [255, 0, 0], 5)

        # self.debug_platform_normal_line = p.addUserDebugLine([0, 0, 0], [0, 1, 0], [255, 0, 0], 5)


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

        #pp.set_joint_positions(self.sim_robot, [2, 3, 4, 5], self.robot_handler.read_cur_conf())
        # self.goal_conf = pp.get_configuration(self.sim_robot)
        # print("current conf: ", self.goal_conf)

        print("[SIM] Successfully initialized PyBullet environment...")

    def update(self, time_elapsed):
        if not self.can_run:
            return

        if not self.pos_probe_command: self.pos_probe_command = ProbePositionSetter(self, pp.get_link_pose(self.sim_robot, 6)[0])
        self.pos_probe_command.onUpdate()

        if not self.pos_plat_command: self.pos_plat_command = PlatformPositionSetter(self, pp.get_joint_position(self.sim_platform, 1), 0)
        self.pos_plat_command.onUpdate()

        if DRAW_TIP_AXES:
            simhelper.draw_tip_axis(self.sim_robot, self.tip_ref_axes)

        # Sync with the main instance and therefore real robot.
        self.time_elapsed = time_elapsed

        if self.can_execute_flow and self.cur_probe_flow:
            self.cur_probe_flow.update(time_elapsed)

        # self.platform_normal = rotate_3d_vector([-1, 0, 0], [0, 0, pp.get_joint_position(self.sim_platform, 1)])
        # p.addUserDebugLine([0, 0, 0], self.platform_normal, [255, 0, 0], 5, replaceItemUniqueId=self.debug_platform_normal_line)

        # Step the simulation
        if p.isConnected(): p.stepSimulation()