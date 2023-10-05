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

URDF_RBT = os.path.join(DATA_DIR, "urdf", "rx200pantex.urdf")
URDF_PLAT = os.path.join(DATA_DIR, "urdf", "actuated_platform.urdf")
URDF_OBJ = os.path.join(DATA_DIR, "urdf", "object.urdf")

class Simulation:
    """
        Simulation class oversees the PyBullet simulation .

        When initialized, a PyBullet client connects to the server, and the robot is simulated.

        Interruptable via. self.can_run
    """

    def __init__(self, parent, robot_offset, object_offset, time_step=1. / UPDATE_RATE):
        self.parent = parent
        self.robot_handler = RobotHandler()

        self.robot_offset = robot_offset
        self.object_offset = object_offset

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
        self.col_flag = False
        self.home_flag = False

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

        # Add robot into PyBullet environment
        self.sim_robot = pp.load_pybullet(URDF_RBT, fixed_base=True, scale=SIM_SCALE)
        p.resetBasePositionAndOrientation(self.sim_robot, self.robot_offset, p.getQuaternionFromEuler(SIM_ROBOT_ORN))
        print(f"[SIM] Initialized robot with ID: {self.sim_robot}")

        # Add center platform AND OBJECT into PyBullet environment
        self.sim_platform = pp.load_pybullet(URDF_PLAT, fixed_base=True, scale=SIM_SCALE)
        p.resetBasePositionAndOrientation(self.sim_platform, SIM_PLATFORM_OFFSET,
                                          p.getQuaternionFromEuler(SIM_PLATFORM_ORN))
        print(f"[SIM] Initialized platform with ID: {self.sim_platform}")

        self.sim_obj = pp.load_pybullet(URDF_OBJ, scale=2)
        p.resetBasePositionAndOrientation(self.sim_obj, np.dot(2, [0, 0, .16]), [0, 0, 0, 1])
        p.createConstraint(parentBodyUniqueId=self.sim_platform, parentLinkIndex=1,
                                            childBodyUniqueId=self.sim_obj,
                                            childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                                            parentFramePosition=self.object_offset, childFramePosition=[0, 0, 0])
        print(f"[SIM] Initialized object with ID: {self.sim_obj} with custom fixed joint!")

        # set camera to center
        pp.set_camera_pose(tuple(np.array((0, 0, 0.25)) + np.array([0.25, -0.25, 0.25])), (0, 0, 0.25))

        # draw axes
        if not PYBULLET_SHOW_GUI:
            simhelper.draw_world_axis(1 * SIM_SCALE, 5)

        # create 3 lines with unique IDs for probe tip axis.
        self.tip_ref_axes = [p.addUserDebugLine([0, 0, 0], [0, 0, 0]),
                             p.addUserDebugLine([0, 0, 0], [0, 0, 0]),
                             p.addUserDebugLine([0, 0, 0], [0, 0, 0])]
        if DRAW_TIP_AXES:
            simhelper.draw_tip_axis(self.sim_robot, self.tip_ref_axes)

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

        if not self.col_flag:
            if self.can_execute_flow and self.cur_probe_flow:
                self.cur_probe_flow.update(time_elapsed)
        elif self.col_flag and not self.home_flag:
            new_point = np.add(pp.get_link_pose(self.sim_robot, 6)[0], [-.1, 0, 0]) # offset probe
            self.pos_probe_command = ProbePositionSetter(self, new_point)
            self.home_flag = True

        if self.col_flag and self.home_flag and self.pos_probe_command.complete:
            print("Going home! [TODO IMPL]")
            #self.pos_probe_command = ProbePositionSetter(self, home_pos)

        # Step the simulation
        if p.isConnected(): p.stepSimulation()

    def shutdown(self):
        self.robot_handler.terminateRobot()