import nidaqmx
from src.sim.simulation import *


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

    def __init__(self, main_instance, port_config, obj_wiz_data):
        # Time Management
        self.update_rate = 1. / UPDATE_RATE
        self.dt = 0
        self.time_ref = 0
        self.time_elapsed = 0

        # Instance management
        self.main_instance = main_instance
        self.simulation_instance = Simulation(main_instance, port_config, obj_wiz_data[0], obj_wiz_data[1])

        self.robot_controller = None
        self.stepper_controller = None
        self.lds_instance = None

        # NIDAQ probing
        print("Connecting to NI-DAQ @ Dev1/ai0... [DISABLED, PLEASE FIX]")
      #  todo self.nidaq_vTask = nidaqmx.Task()
      #  self.nidaq_vTask.ai_channels.add_ai_voltage_chan("Dev1/ai0")
        self.probe_voltage = 0

        # Program mode
        self.canRun = True

        # Temp test stuff:
        self.obj_distance = 0
        self.obj_height = None # obj_wiz_data[2]

    def send_update(self):
        if not self.canRun:
            return

        # ('Tik')
        self.time_ref = time.time()

        # Update robot/simulation
        self.simulation_instance.update(self.time_elapsed)

        # Collision check
        closest_points = p.getClosestPoints(self.simulation_instance.sim_robot, self.simulation_instance.sim_platform,
                                            0.2)
        if closest_points:
            self.obj_distance = min(point[8] for point in closest_points)
        else:
            self.obj_distance = 0.2

        # Retrieve probe voltage
        # todo self.probe_voltage = self.nidaq_vTask.read()

        # Time management ('Tok')
        time.sleep(self.update_rate)
        self.dt = time.time() - self.time_ref
        self.time_elapsed += self.dt

    def shutdown(self):
        """
            Function that will properly disable the robot and disconnect from the simulation.
            IF this is not called at end of program life, then the robot will NOT return to home position.
        """
        self.simulation_instance.shutdown()
        if p.isConnected(): p.disconnect()
        print("Shutdown!")
        self.canRun = False
