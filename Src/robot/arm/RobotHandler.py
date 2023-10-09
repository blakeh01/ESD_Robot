from Src.robot.arm.robohelper import *
from Src.robot.SerialMonitor import *


class RobotHandler:

    def __init__(self, port_config, dummy=False):
        self.stepper_board = StepperHandler(port_config.stepper_port, port_config.stepper_baud)
        self.feather0 = SerialMonitor(port_config.feather_port, port_config.feather_baud)

        # Time management
        self.time_alive = 0

        # Conncet to device
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

        self.i = 0

    def update(self):
        self.i += 1

        if i >= 3:
            print(self.read_cur_conf()[0])
            self.i = 0

    def set_goal_conf(self, joint_states):
        # Simulation rotation -> DXL position (add pi to joint state, turn into degrees, divide by position unit per deg)
        rail_pos = radiansToDxlUnits(joint_states[0])
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

        self.stepper_board.write_b(int(rail_pos), 3600)

        # Syncwrite goal positions
        self.pos_group_writer.txPacket()  # begins motion of the DXLs

        # Clear syncwrite parameter storage
        self.pos_group_writer.clearParam()

    def read_cur_conf(self):
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
            conf.append(
                self.group_bulk_read_pos.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            )

        # convert to joint states, remove shadow joint
        conf = [dxlToRadians(pos, -np.pi) for pos in conf]
        del conf[2]

        # insert stepper pos @ 0
        conf.insert(0, self.stepper_board.rail_pos / 1000)

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
        self.stepper_board.close()

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
