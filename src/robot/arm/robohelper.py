'''

    robohelper.py implements various functions to communicate with the dynamixel_sdk and ultimately with the modified
     RX200 arm

     See the dynamixel SDK to learn more about what group parameters and any other useful functions.
     https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_read_write_protocol_2_0/#python-read-write-protocol-20

    Assumptions when using these function: All connected Dynamixels are XL430s. Will include variability if needed.

'''

import os

import numpy as np
from dynamixel_sdk import *

from src.robot.arm.rbt_constants import *


def dxlToRadians(value, radian_offset):
    '''
        Converts the dynamixel position value to rotational position in radians.

    :param value: dynamixel position
    :param radian_offset: offset in radians
    :return: rotational position from dynamixel position
    '''
    return np.radians(value * 0.087890625) + radian_offset


def radiansToDxlUnits(value):
    '''
        Converts rotational position in radians to dynamixel postions.

    :param value: rotation in radians
    :return: dynamixel position
    '''
    return np.degrees(value) / 0.087890625


def writeGoalPosition(packet_handler, port_handler: PortHandler, id, position):
    '''
        Writes to the goal position address a specified goal position

    :param packet_handler: dynamixel controller packet handler
    :param port_handler: dynamixel controller port handler
    :param id: id of the dynamixel to set goal position
    :param position: position (in dynamixel units) to set the goal to
    '''
    packet_handler.write4ByteTxRx(port_handler, id, ADDR_GOAL_POSITION, position)


def writeDataAndWait4Byte(packet_handler: PacketHandler, port_handler: PortHandler, id, addr, data):
    packet_handler.write4ByteTxRx(port_handler, id, addr, data)
    time.sleep(0.01)


def writeDataAndWait2Byte(packet_handler: PacketHandler, port_handler: PortHandler, id, addr, data):
    packet_handler.write2ByteTxRx(port_handler, id, addr, data)
    time.sleep(0.01)


def setTorques(packet_handler, port_handler, id, torque):
    '''
        Set the torque control on a specified dynamixel id (or ids)

    :param packet_handler: dynamixel controller packet handler
    :param port_handler: dynamixel controller port handler
    :param dxl_ids: id of the dynamnixel to set torque
    :param torque: 1 - torque on, 0 - torque off (USE RBTCONTS CLASS)
    '''
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, id, ADDR_TORQUE_ENABLE, torque)
    if not statusCheck(packet_handler, dxl_comm_result, dxl_error):
        print(f"Failed to enable torque on DXL ID{id}!")
        quit()


def byteIntegerTransform(number):
    '''
        Transforms an integer position/velocity into 4 bytes of data.
    :param number: to transmute
    :return: byte array
    '''
    return [DXL_LOBYTE(DXL_LOWORD(number)), DXL_HIBYTE(DXL_LOWORD(number)),
            DXL_LOBYTE(DXL_HIWORD(number)), DXL_HIBYTE(DXL_HIWORD(number))]


def addGroupParameter(groupSyncWrite, dxl_id, data):
    '''
        Writes to a group writer a specific parameter
        Once all parameters are added, a seperate function can be run to execeute that parameter
        across all dynamixels simultaneously.

    :param groupSyncWrite: group writer
    :param dxl_id: dynaimxel id to receive the parameter
    :param data: data of the parameter
    '''
    dxl_addparam_result = groupSyncWrite.addParam(dxl_id, data)
    if not dxl_addparam_result:
        print(f"[ID:{dxl_id}] groupSyncWrite addparam failed")
        quit()


def statusCheck(packet_handler, comm_result, error):
    '''
        Abstracts the status check functionality to ensure no errors are thrown during reading/writing to dynamixels.
    '''
    if comm_result != COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(comm_result))
        return False
    elif error != 0:
        print("%s" % packet_handler.getRxPacketError(error))
        return False
    return True


#  Console Controls

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
