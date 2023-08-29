import pybullet as p
import pybullet_planning as pp
import time

from Src.sim.sim_constants import *
from Src.robot.RobotHandler import *
from Src.sim.simulation import *

import nidaqmx
import nidaqmx.system

class Controller:
    """
        Initializes the overseer class.

        Contains 3 instances:
            * Robot instance (self.robot_instance)
                - In charge of the low-level control to the robot assembly (arm, stepper motor)
            * Simulation instance (self.simulation_instance)
                - Contains code to properly run simulation and simulation position references.
            * GUI instance (self.gui)
                - Allows for user interaction and viewing of data.
                - Will most likely need to execute on separate thread

        This class also is in control of time, using it's time to pass to all 3 of the instances.
        This ensures that each instance has a reference to each other's time.

        Upon initialization, the simulation joint states are set to the REAL robot joint states.
        Once initialized, the code will run an interruptable infinite loop (to halt, set self.can_run = false) that
        will update each instance class, as well as provide time data.
    """

    def __init__(self, main_instance):
        # Time Management
        self.update_rate = 1. / UPDATE_RATE
        self.dt = 0
        self.time_ref = 0
        self.time_elapsed = 0

        # Instance management todo: disabled robot handler... enable to do actual tests...
        # self.robot_instance = RobotHandler()
        # self.maneuver_handler = ManeuverHandler(self.robot_instance)
        self.simulation_instance = Simulation()
        self.main_instance = main_instance

        # NIDAQ probing
        print("Connecting to NI-DAQ @ Dev1/ai0...")
        self.nidaq_vTask = nidaqmx.Task()
        self.nidaq_vTask.ai_channels.add_ai_voltage_chan("Dev1/ai0")
        self.probe_voltage = 0

        # Program mode
        self.canRun = True

        # Temp test stuff:
        self.obj_distance = 0

    def send_update(self):
        """
            A synchronized update function:
                - Keeps track of 'real' time elapsed
                - Sleeps at some arbitrary update rate.
        """
        if not self.canRun:
            return

        # ('Tik')
        self.time_ref = time.time()

        # Update robot/simulation
        # self.robot_instance.update_robot()
        self.simulation_instance.update_simulation(self.time_elapsed)
        #self.maneuver_handler.update()

        # Collision check
        dist_list = []
        for point in p.getClosestPoints(self.simulation_instance.sim_robot, self.simulation_instance.sim_platform, 0.2):
            dist_list.append(point[8])
        if len(dist_list) > 0:
            self.obj_distance = min(dist_list)
        else:
            self.obj_distance = 0.2

        # Retrieve probe voltage
        self.probe_voltage = self.nidaq_vTask.read()

        # Time management ('Tok')
        time.sleep(self.update_rate)
        self.dt = time.time() - self.time_ref
        self.time_elapsed += self.dt

    def shutdown(self):
        """
            Function that will properly disable the robot and disconnect from the simulation.
            IF this is not called at end of program life, then the robot will NOT return to home position.
        """
        # self.robot_instance.terminateRobot()
        p.disconnect()
        print("Shutdown!")
        self.canRun = False

    def restart_sim(self):
        self.canRun = False
        self.main_instance.window.setParent(None)
        p.disconnect()

        time.sleep(1)
        del self.simulation_instance
        time.sleep(1)

        self.simulation_instance = Simulation()
        self.main_instance.embed_pysim()
        self.canRun = True
