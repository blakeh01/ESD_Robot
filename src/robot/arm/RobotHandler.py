from src.robot.arm.robohelper import *
from src.robot.SerialMonitor import *


class RobotHandler:
    """
        This class encompasses the entire 'real' robotic manipulator.

        Features:
            - DXL position management
            - Stepper motor management
            - Feather and other device management
            - Serial control
            - A few common sequences
            - Maneuver control

        The stepper motor is controlled using a class named StepperHandler.

        Note: Shoulder consists of 2 motors, however, motor 3 is shadowing motor 2. This means we can write commands
        to motor 2 and motor 3 will simply match it.
    """

    def __init__(self, port_config, stepper_controller: StepperHandler, dummy=False):
        # Time management
        self.time_alive = 0

        self.stepper_controller = stepper_controller

        # Connect to device
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.port_handler = PortHandler(port_config.rbt_port)
        self.pos_group_writer = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_POSITION,
                                               LEN_GOAL_POSITION)
        self.group_bulk_read_pos = GroupBulkRead(self.port_handler, self.packet_handler)
        self.group_bulk_read_load = GroupBulkRead(self.port_handler, self.packet_handler)

        # Open port, terminate if no connection is established.
        try:
            if self.port_handler.openPort():
                print(f"[DXL] Successfully opened port on {port_config.rbt_port}!")
            else:
                input(f"[DXL] Failed to open port on {port_config.rbt_port}.")
                quit()

            # Set the baud rate, terminate under failure.
            if self.port_handler.setBaudRate(port_config.rbt_baud):
                print(f"[DXL] Successfully set baud rate to {port_config.rbt_baud} bps!")
            else:
                input(f"[DXL] Failed to set baud rate.")
                quit()
        except:
            input("Error connecting to robotic arm! Ensure proper connections are made to the computer!")
            quit()

        # Set up motors
        self.motors = []
        for i in DXL_IDS:
            m = DXL_Motor(i, self.packet_handler, self.port_handler)
            m.set_torque(TORQUE_DISABLE if dummy else TORQUE_ENABLE)
            self.motors.append(m)

        for i in range(1, 6):
            result = self.group_bulk_read_pos.addParam(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            print(f"[ROBOT] Created bulk read parameter for motor ID#{i}. (Success? {result})")

        for i in range(1, 6):
            result = self.group_bulk_read_load.addParam(i, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)
            print(f"[ROBOT] Created bulk read parameter for load on motor ID#{i}. (Success? {result})")

        # set some default params:
        # ID02/03
        writeDataAndWait4Byte(self.packet_handler, self.port_handler, 2, ADDR_PROF_VELOCITY, 30)
        writeDataAndWait4Byte(self.packet_handler, self.port_handler, 2, ADDR_PROF_ACCEL, 10)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 2, ADDR_POS_P_GAIN, 640)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 2, ADDR_POS_I_GAIN, 400)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 2, ADDR_POS_D_GAIN, 3600)

        # ID04
        writeDataAndWait4Byte(self.packet_handler, self.port_handler, 4, ADDR_PROF_VELOCITY, 30)
        writeDataAndWait4Byte(self.packet_handler, self.port_handler, 4, ADDR_PROF_ACCEL, 10)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 4, ADDR_POS_P_GAIN, 4000)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 4, ADDR_POS_I_GAIN, 600)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 4, ADDR_POS_D_GAIN, 4500)

        # ID05
        writeDataAndWait4Byte(self.packet_handler, self.port_handler, 5, ADDR_PROF_VELOCITY, 40)
        writeDataAndWait4Byte(self.packet_handler, self.port_handler, 5, ADDR_PROF_ACCEL, 30)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 5, ADDR_POS_P_GAIN, 2500)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 5, ADDR_POS_I_GAIN, 1200)
        writeDataAndWait2Byte(self.packet_handler, self.port_handler, 5, ADDR_POS_D_GAIN, 3600)

        self.update_counter = 0

    def update(self):
        self.torque_collision_check()

    def torque_collision_check(self):
        self.update_counter += 1

        if self.update_counter >= 3:
            read_data = self.read_cur_conf()
            self.update_counter = 0

            if read_data is not None:
                if 30 < read_data[0][0] < 60000:
                    print("COLLISION DETECTED!")

    def set_goal_conf(self, joint_states):
        """
        Using the DynamixelSDK to write to the goal position register of the servos.

        @param joint_states: Simulated joint state values, ultimately converted to dxl units before writing to servos.
        """

        # get DXL units for each joint state. Revolute joints are + np.pi due to PyBullet -pi -> pi instead of
        # dynamixels 0 -> 2pi.
        rail_pos = radiansToDxlUnits(joint_states[0])
        shoulder_pos = radiansToDxlUnits(joint_states[2] + np.pi)
        elbow_pos = radiansToDxlUnits(joint_states[3] + np.pi)
        wrist_pos = radiansToDxlUnits(joint_states[4] + np.pi)

        # add a group write parameter containing the positional data to the DXLs
        addGroupParameter(self.pos_group_writer, DXL_IDS[1], byteIntegerTransform(int(shoulder_pos)))
        addGroupParameter(self.pos_group_writer, DXL_IDS[2], byteIntegerTransform(int(shoulder_pos)))
        addGroupParameter(self.pos_group_writer, DXL_IDS[3], byteIntegerTransform(int(elbow_pos)))
        addGroupParameter(self.pos_group_writer, DXL_IDS[4], byteIntegerTransform(int(wrist_pos)))

        # use stepper controller to write to linear rail
        self.stepper_controller.write_linear_rail(int(rail_pos), 2500)

        # Syncwrite goal positions
        self.pos_group_writer.txPacket()  # begins motion of the DXLs

        # Clear syncwrite parameter storage
        self.pos_group_writer.clearParam()

    def read_cur_conf(self):
        """
        Uses the bulk readers initialized in init to quickly read position and force data

        @return: [forces, positions] where each is a 5 element array corresponding to each motor.
        """
        conf = []

        # read current robot position
        res = self.group_bulk_read_pos.txRxPacket()
        if res != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(res))
            return

        for i in DXL_IDS:
            res = self.group_bulk_read_pos.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if not res:
                print(f"Failed to read motor ID:{i}!")
                return
            conf.append(
                self.group_bulk_read_pos.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            )

        # convert to joint states, remove shadow joint
        conf = [dxlToRadians(pos, -np.pi) for pos in conf]
        del conf[2]

        # insert stepper pos @ 0
        conf.insert(0, self.stepper_controller.rail_pos / 1000)

        forces = []
        # read current robot loads
        res = self.group_bulk_read_load.txRxPacket()
        if res != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(res))
            return

        for i in DXL_IDS:
            res = self.group_bulk_read_load.isAvailable(i, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)
            if not res:
                print(f"Failed to read motor ID:{i}!")
                return
            forces.append(
                self.group_bulk_read_load.getData(i, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD)
            )

        return [forces, conf]

    def terminate_robot(self):
        print("[RBT] Shutting down... TORQUE is now OFF!")
        for m in self.motors:
            time.sleep(0.05)
            m.set_torque(TORQUE_DISABLE)

        self.port_handler.closePort()


class DXL_Motor:

    def __init__(self, id, packet_handler: PacketHandler, port_handler: PortHandler):
        self.id = id
        self.goal_pos = 0
        self.cur_pos = 0

        self.packet_handler = packet_handler
        self.port_handler = port_handler

    def update(self):
        pass

    def set_torque(self, en):
        setTorques(self.packet_handler, self.port_handler, self.id, en)

    def set_goal(self):
        pass

    def get_cur_pos(self):
        pass

    def get_goal(self):
        pass
