from Src.robot.robohelper import *
from Src.robot.stepperhandler import *


class RobotHandler:
    """
        This class encompasses the entire 'real' robotic manipulator.

        Features:
            - DXL position management
            - Stepper motor management
            - Serial control
            - A few common sequences
            - Maneuver control
            - Status management (WIP)
            - Event management (WIP)

        The stepper motor is controlled using a class named StepperHandler.

        Note: Shoulder consists of 2 motors, however, motor 3 is shadowing motor 2. This means we can write commands
        to motor 2 and motor 3 will simply match it.
    """

    def __init__(self):
        # Position management
        self.WRITABLE_MOTOR_IDS = [1, 2, 4, 5]
        self.cached_pos = []
        self.stepper_handler = StepperHandler()

        # Maneuver management
        self.goal_conf = None
        self.maneuver_list = None  # a list of confs to reach
        self.current_plat_rot = 0  # the current center platform rotation.

        # Status management
        self.status = RBT_STATUS_WARN

        # Time management
        self.time_alive = 0

        # Device management
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.port_handler = PortHandler(DEVICENAME)
        self.pos_group_writer = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_POSITION,
                                               LEN_GOAL_POSITION)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)

        print(f"Initializing robot...")

        # V Connect to DXLs V

        # Open port, terminate if no connection is established.
        if self.port_handler.openPort():
            print(f"Successfully opened port on {DEVICENAME}!")
        else:
            print(f"Failed to open port on {DEVICENAME}.")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set the baud rate, terminate under failure.
        if self.port_handler.setBaudRate(BAUDRATE):
            print(f"Successfully set baud rate to {BAUDRATE} bps!")
        else:
            print(f"Failed to set baud rate.")
            print("Press any key to terminate...")
            getch()
            quit()

        self.startRobot()

        print(f"Finished initialization!")
        # if no errors were created during initialization, set status to OKAY
        if self.status != RBT_STATUS_ERROR:
            self.status = RBT_STATUS_OKAY

        self.i = 0

    def startRobot(self):
        """
            Robot start sequence:
            Enables all DXL motors and the stepper motor and reads their position to memory.
        """
        # Enable all the motors
        self.writeMotorTorque(TORQUE_ENABLE)

        # Create a new bulk read parameter for reading the current positions of the DXLs
        for i in range(1, 6):
            result = self.group_bulk_read.addParam(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            print(f"[ROBOT] Created bulk read parameter for motor ID#{i}. (Success? {result})")
            if not result:
                self.status = RBT_STATUS_ERROR

        # Test read the robot positions
        self.getRobotPositionArray()

    def updateRobot(self, time_alive, step):
        """
            Update the robot (and stepper motor)
        """
        self.i += 1

        if self.i % 2 == 0:
            self.getRobotPositionArray()


        self.time_alive = time_alive
        self.stepper_handler.update_interpolated_positions(step)

    def write_joint_states_to_robot(self, joint_state, rail_states=None):
        """
           Given a properly formatted joint_state (matches same format read from the simulation)
           write it to the motor and stepper motor.
        """
        if len(joint_state) != 5:
            print("Tried to write an invalid joint state!")
            return

        #self.stepper_handler.write_3axis(rail_states)

        # Simulation rotation -> DXL position (add pi to joint state, turn into degrees, divide by position unit per deg)
        waist_pos = radiansToDxlUnits(joint_state[1] + np.pi)
        shoulder_pos = radiansToDxlUnits(joint_state[2] + np.pi)
        elbow_pos = radiansToDxlUnits(joint_state[3] + np.pi)
        wrist_pos = radiansToDxlUnits(joint_state[4] + np.pi)
        waist_pos = 2005 # todo care

        # add a group write parameter containing the positional data to the DXLs
        addGroupParameter(self.pos_group_writer, DXL_ID_01, byteIntegerTransform(int(waist_pos)))
        addGroupParameter(self.pos_group_writer, DXL_ID_02, byteIntegerTransform(int(shoulder_pos)))
        addGroupParameter(self.pos_group_writer, DXL_ID_03, byteIntegerTransform(int(shoulder_pos)))
        addGroupParameter(self.pos_group_writer, DXL_ID_04, byteIntegerTransform(int(elbow_pos)))
        addGroupParameter(self.pos_group_writer, DXL_ID_05, byteIntegerTransform(int(wrist_pos)))

        # Syncwrite goal position
        self.pos_group_writer.txPacket()  # begins motion of the DXLs
        self.stepper_handler.linear_rail_motor.add_position_to_queue(1000 * joint_state[0])  # begins motion of the stepper motor

        # Clear syncwrite parameter storage
        self.pos_group_writer.clearParam()

    def getRobotPositionArray(self):
        """
            Returns the robot position array:
            [
                0: Stepper position,
                1: Waist position,
                2: Shoulder position,
                3: Elbow position,
                4: wrist position
            ]
            Also sets the cached position array. do NOT call twice within a very short time span.
            If position is needed, accessed the cached position - this is updated automatically.
        """
        position_array = []

        # read rail position
        position_array.append(self.stepper_handler.linear_rail_motor.get_position())

        # read robotic arm DXLs
        res = self.group_bulk_read.txRxPacket()
        if res != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(res))
            self.status = RBT_STATUS_ERROR
            return

        for i in self.WRITABLE_MOTOR_IDS:
            res = self.group_bulk_read.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if not res:
                print(f"Failed to read motor ID:{i}!")
                self.status = RBT_STATUS_ERROR
                return
            position_array.append(
                self.group_bulk_read.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            )
        self.cached_pos = position_array

        return position_array

    def get_conf(self):
        """
            Converts the robot position array to the current joint configuration.
        """
        raw = self.cached_pos
        joint_states = []

        if raw is None or len(raw) != 5:
            return

        # converts stepper motor to simulated units (in mm in real world, divide by 1000 to turn into m)
        joint_states.append(raw[0] / 1000)

        # converts the DXL robotic arm to simulated units
        for i in range(1, 5):
            joint_states.append(dxlToRadians(raw[i], -np.pi))

        return joint_states

    def writeMotorTorque(self, value, index=-1):
        """
            Enables torque on the specified motor index. (index -1 writes all motors)
            Otherwise, provide an index that corresponds to the DXL_id.
        """
        if index == -1:
            for i in range(1, 6):
                setTorque(self.packet_handler, self.port_handler, i, value)
            return

        setTorque(self.packet_handler, self.port_handler, index, value)

    def terminateRobot(self):
        """
            Properly disables robot: disables torque, disconnects ports, resets stepper to home.
        """
        self.writeMotorTorque(TORQUE_DISABLE)
        self.status = RBT_STATUS_ERROR
        self.port_handler.closePort()
        self.stepper_handler.close()

    def get_rbt_status(self):
        return self.status
