import os.path
import time

import src.sim.simhelper as simhelper
from src.robot.arm.RobotHandler import RobotHandler
from src.sim.Command import *
from src.sim.sim_constants import *
from src.util.math_util import *

# Paths for URDF files.
DATA_DIR = os.path.join(os.path.abspath('../'), "data", "sim")

URDF_RBT = os.path.join(DATA_DIR, "urdf", "rx200pantex.urdf")
URDF_PLAT = os.path.join(DATA_DIR, "urdf", "actuated_platform.urdf")


class Simulation:
    """
        Simulation class oversees the PyBullet simulation .

        When initialized, a PyBullet client connects to the server, and the robot is simulated.

        Interruptable via. self.can_run
    """

    def __init__(self, controller, port_config, robot_offset, object_offset, time_step=1. / UPDATE_RATE):
        self.controller = controller
        self.port_config = port_config

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
        self.lineup_normal = [-1, 0, 0]  # direction of the normal vector that 'lines up' the platform to the robot.
        self.col_flag = False
        self.home_flag = False
        self.home_conf = None
        self.probe_home = None

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

        # SET HOME POSITION TO DEFAULT CONF.
        # self.home_conf = self.robot_handler.read_cur_conf()[1]
        # self.home_conf = np.add(self.home_conf, [0, 0, 0.3, 0, 0])

        self.drive_motors_to_home()
        for i in range(100):
            p.stepSimulation()
            time.sleep(0.01)

        self.probe_home = pp.get_link_pose(self.sim_robot, 6)[0]

        print("[SIM] Successfully initialized PyBullet environment...")

    def update(self, time_elapsed):
        if not self.can_run:
            return

        # todo: self.robot_handler.update()

        if not self.pos_probe_command: self.pos_probe_command = ProbePositionSetter(self, self.probe_home, [0, 0, 0])
        self.pos_probe_command.onUpdate()

        if not self.pos_plat_command: self.pos_plat_command = PlatformPositionSetter(self, pp.get_joint_position(
            self.sim_platform, 1), 0)
        self.pos_plat_command.onUpdate()

        if DRAW_TIP_AXES:
            simhelper.draw_tip_axis(self.sim_robot, self.tip_ref_axes)

        # Sync with the main instance and therefore real robot.
        self.time_elapsed = time_elapsed

        if not self.col_flag:
            if self.can_execute_flow and self.cur_probe_flow:
                self.cur_probe_flow.update(time_elapsed)
        elif self.col_flag and not self.home_flag:
            print("[SIM] Collision detected!")
            new_point = np.add(pp.get_link_pose(self.sim_robot, 6)[0], [-.1, 0, 0])  # offset probe
            self.pos_probe_command = ProbePositionSetter(self, new_point, [0, 0, 0])
            self.home_flag = True

        if self.col_flag and self.home_flag:
            print("[SIM] Going home!")
            self.parent.sim_stop()
            self.parent.lbl_rbt_status.setText("Collision Error!")

        # Step the simulation
        if p.isConnected(): p.stepSimulation()

    def drive_motors_to_home(self):
        return # todo
        # RAIL
        p.setJointMotorControl2(self.sim_robot, 1,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.home_conf[0])

        # WAIST
        p.setJointMotorControl2(self.sim_robot, 2,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.home_conf[1])

        # SHOULDER
        p.setJointMotorControl2(self.sim_robot, 3,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.home_conf[2])

        # ELBOW
        p.setJointMotorControl2(self.sim_robot, 4,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.home_conf[3])

        # WRIST/PROBE
        p.setJointMotorControl2(self.sim_robot, 5,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.home_conf[4])

    def shutdown(self):
        self.parent.sim_stop()
        self.robot_handler.terminate_robot()
