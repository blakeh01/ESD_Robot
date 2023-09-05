import os.path

import Src.sim.simhelper as simhelper
import numpy as np
import pybullet as p
import pybullet_planning as pp
from Src.sim.sim_constants import *

import math

# Paths for URDF files.
DATA_DIR = os.path.join(os.path.abspath('../'), "Data", "sim")

URDF_PLANE = os.path.join(DATA_DIR, "urdf", "plane.urdf")
URDF_RBT = os.path.join(DATA_DIR, "urdf", "rx200pantex.urdf")
URDF_PLAT = os.path.join(DATA_DIR, "urdf", "actuated_platform.urdf")
URDF_OBJECT = os.path.join(DATA_DIR, "urdf", "object.urdf")


# URDF_CHAMBER = os.path.join(DATA_DIR, "urdf", "chamber.urdf")
# URDF_SCANNER = os.path.join(DATA_DIR, "urdf", "scanner.urdf")


class Simulation:
    """
        Simulation class oversees the PyBullet simulation .

        When initialized, a PyBullet client connects to the server, and the robot is simulated.

        Interruptable via. self.can_run
    """

    def __init__(self, time_step=1. / UPDATE_RATE):
        # Time management
        self.time_elapsed = 0
        self.time_step = time_step

        # Simulation IDs
        self.sim_id = -1
        self.sim_robot = -1
        self.sim_platform = -1
        self.sim_object = -1
        self.sim_chamber = -1
        self.sim_scanner = -1

        # Simulation ref
        self.robot_joints = [0, 2, 3, 4, 5]
        self.current_point_cloud = None
        self.probe_vector = [0, 0, 0]  # describes the vector to line up platform for probing. todos
        self.plat_rot = 0

        # Debug stuff
        self.tip_ref_axes = []
        self.debug_point = None
        self.debug_point_cloud = None

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
        p.setCollisionFilterPair(self.sim_chamber, self.sim_robot, -1, -1, 0)
        p.setCollisionFilterPair(self.sim_chamber, self.sim_platform, -1, -1, 0)

        pp.set_camera_pose(tuple(np.array((0, 0, 0.1)) + np.array([0.25, -0.25, 0.25])), (0, 0, 0.1))

        if not PYBULLET_SHOW_GUI:
            simhelper.draw_world_axis(1 * SIM_SCALE, 5)

        # create 3 lines with unique IDs for probe tip axis.
        self.tip_ref_axes = [p.addUserDebugLine([0, 0, 0], [0, 0, 0]),
                             p.addUserDebugLine([0, 0, 0], [0, 0, 0]),
                             p.addUserDebugLine([0, 0, 0], [0, 0, 0])]
        if DRAW_TIP_AXES:
            simhelper.draw_tip_axis(self.sim_robot, self.tip_ref_axes)

        self.debug_point = p.addUserDebugPoints([[0, 0, 0]], [[255, 0, 0]], 0)
        self.current_point_cloud = None

        self.probe_enable = False
        self.slice_idx = 0
        self.point_idx = 0

        print("[SIM] Successfully initialized PyBullet environment...")

    def update_simulation(self, time_elapsed):
        if not self.can_run:
            return

        if DRAW_TIP_AXES:
            simhelper.draw_tip_axis(self.sim_robot, self.tip_ref_axes)

        if self.current_point_cloud is not None:

            if self.probe_enable:
                self.do_probing()
            else:
                self.idx_slice = 0

        # Sync with the main instance and therefore real robot.
        self.time_elapsed = time_elapsed

        # Step the simulation
        p.stepSimulation()


    def do_probing(self):
        _slices = list(self.current_point_cloud.sliced_points.items())

        if 0 <= self.slice_idx < len(_slices):
            z_value, points = _slices[self.slice_idx]

        self.probe_enable = False
    def write_joint_states(self, joint_state):
        if joint_state == self.get_sim_joint_states():
            return

        p.resetJointState(self.sim_robot, 0, joint_state[0])
        for i in range(2, 6):
            p.resetJointState(self.sim_robot, i,
                              joint_state[i - 1])

    def get_sim_joint_states(self):
        """
            Returns the 'important' joint states.

            Revolute joints are defined on -pi -> pi, bounded by min/max rotations set in URDF file. (radians)
            Prismatic joints are defined purely based on the bounds in the min/max positions in the URDF file. (meters)

            return joint state indexes:
            0 -> linear actuator  ,  1->waist  ,   2->shoulder   ,   3->elbow    ,   4-> wrist
        """
        return [p.getJointState(self.sim_robot, 0)[0], p.getJointState(self.sim_robot, 2)[0],
                p.getJointState(self.sim_robot, 3)[0], p.getJointState(self.sim_robot, 4)[0],
                p.getJointState(self.sim_robot, 5)[0]]
