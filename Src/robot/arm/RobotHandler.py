from Src.robot.arm.robohelper import *
from Src.robot.SerialMonitor import SerialMonitor, StepperHandler, LDS

class RobotHandler:

    def __init__(self):
        #self.stepper_board = StepperHandler(STEPPER_PORT, STEPPER_BAUD)
        #self.feather0 = SerialMonitor()

        # Time management
        self.time_alive = 0

        # Conncet to device
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.port_handler = PortHandler(DEVICENAME)
        self.pos_group_writer = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_POSITION,
                                               LEN_GOAL_POSITION)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)

        # Open port, terminate if no connection is established.
        try:
            if self.port_handler.openPort():
                print(f"[DXL] Successfully opened port on {DEVICENAME}!")
            else:
                input(f"[DXL] Failed to open port on {DEVICENAME}.")
                quit()

            # Set the baud rate, terminate under failure.
            if self.port_handler.setBaudRate(BAUDRATE):
                print(f"[DXL] Successfully set baud rate to {BAUDRATE} bps!")
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
            m.set_torque(TORQUE_ENABLE)
            self.motors.append(m)

        for i in range(1, 6):
            result = self.group_bulk_read.addParam(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            print(f"[ROBOT] Created bulk read parameter for motor ID#{i}. (Success? {result})")

        # set some default params:
        #ID02/03
        writeDataAndWait(self.packet_handler, self.port_handler, 2, ADDR_PROF_VELOCITY, 75)
        writeDataAndWait(self.packet_handler, self.port_handler, 2, ADDR_PROF_ACCEL, 20)
        # writeDataAndWait(self.packet_handler, self.port_handler, 2, ADDR_POS_P_GAIN, 9000)
        # writeDataAndWait(self.packet_handler, self.port_handler, 2, ADDR_POS_I_GAIN, 1500)
        # writeDataAndWait(self.packet_handler, self.port_handler, 2, ADDR_POS_D_GAIN, 3600)

        #ID04
        writeDataAndWait(self.packet_handler, self.port_handler, 4, ADDR_PROF_VELOCITY, 75)
        writeDataAndWait(self.packet_handler, self.port_handler, 4, ADDR_PROF_ACCEL, 20)
        # writeDataAndWait(self.packet_handler, self.port_handler, 4, ADDR_POS_P_GAIN, 6000)
        # writeDataAndWait(self.packet_handler, self.port_handler, 4, ADDR_POS_I_GAIN, 600)
        # writeDataAndWait(self.packet_handler, self.port_handler, 4, ADDR_POS_D_GAIN, 3600)

        #ID05
        writeDataAndWait(self.packet_handler, self.port_handler, 5, ADDR_PROF_VELOCITY, 75)
        writeDataAndWait(self.packet_handler, self.port_handler, 5, ADDR_PROF_ACCEL, 30)
        # writeDataAndWait(self.packet_handler, self.port_handler, 5, ADDR_POS_P_GAIN, 1600)
        # writeDataAndWait(self.packet_handler, self.port_handler, 5, ADDR_POS_I_GAIN, 600)
        # writeDataAndWait(self.packet_handler, self.port_handler, 5, ADDR_POS_D_GAIN, 3600)

    def update(self):
        pass

    def set_goal_conf(self, joint_states):
        # Simulation rotation -> DXL position (add pi to joint state, turn into degrees, divide by position unit per deg)
        print(joint_states)
        waist_pos = radiansToDxlUnits(joint_states[1] + np.pi)
        shoulder_pos = radiansToDxlUnits(joint_states[2] + np.pi)
        elbow_pos = radiansToDxlUnits(joint_states[3] + np.pi)
        wrist_pos = radiansToDxlUnits(joint_states[4] + np.pi)
        waist_pos = 2005  # todo care

        # add a group write parameter containing the positional data to the DXLs
        addGroupParameter(self.pos_group_writer, DXL_IDS[0], byteIntegerTransform(int(waist_pos)))
        addGroupParameter(self.pos_group_writer, DXL_IDS[1], byteIntegerTransform(int(shoulder_pos)))
        addGroupParameter(self.pos_group_writer, DXL_IDS[2], byteIntegerTransform(int(shoulder_pos)))
        addGroupParameter(self.pos_group_writer, DXL_IDS[3], byteIntegerTransform(int(elbow_pos)))
        addGroupParameter(self.pos_group_writer, DXL_IDS[4], byteIntegerTransform(int(wrist_pos)))

        # todo write to stepper

        # Syncwrite goal positions
        self.pos_group_writer.txPacket()  # begins motion of the DXLs

        # Clear syncwrite parameter storage
        self.pos_group_writer.clearParam()

    def read_cur_conf(self):
        conf = []

        # read current robot position
        res = self.group_bulk_read.txRxPacket()
        if res != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(res))
            return

        for i in DXL_IDS:
            res = self.group_bulk_read.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if not res:
                print(f"Failed to read motor ID:{i}!")
            conf.append(
                self.group_bulk_read.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            )

        # convert to joint states, remove shadow joint
        conf = [dxlToRadians(pos, np.pi) for pos in conf]
        del conf[2]

        # todo read stepper motor position
        #conf.insert(0, stepper_pos)

        return conf

    def get_goal_positions(self):
        def are_close(current_pos, goal_pos, tolerance=5):
            return abs(current_pos - goal_pos) <= tolerance

        goal_positions = []

        # Get the current positions
        current_positions = self.read_cur_conf()

        # Simulation rotation -> DXL position (add pi to joint state, turn into degrees, divide by position unit per deg)
        goal_positions.append(are_close(radiansToDxlUnits(joint_state[1] + np.pi), current_positions[0]))
        goal_positions.append(are_close(radiansToDxlUnits(joint_state[2] + np.pi), current_positions[1]))
        goal_positions.append(are_close(radiansToDxlUnits(joint_state[2] + np.pi), current_positions[2]))
        goal_positions.append(are_close(radiansToDxlUnits(joint_state[3] + np.pi), current_positions[3]))
        goal_positions.append(are_close(radiansToDxlUnits(joint_state[4] + np.pi), current_positions[4]))

        return goal_positions

    def terminateRobot(self):
        print("[RBT] Shutting down... TORQUE is now OFF! [Please implement going home :)]")
        for m in self.motors:
            time.sleep(0.05)
            m.set_torque(TORQUE_DISABLE)

        self.port_handler.closePort()
       # self.stepper_handler.close()

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
