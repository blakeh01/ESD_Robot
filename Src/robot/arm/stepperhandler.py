'''

    This class needs work.

    StepperHandler keeps track of the linear rail (and maybe 3d axis system) and sends gcode to update position

    The current interpolator is broken, and does not function as intended.

    The position needs to be interpolated as the 3d printer controller does NOT report back the current
    stepper position.

    Also... Gcode is meant to be preprocessed and execute in parallel with the controls.
    Which is the ultimate goal, so this class might be obsolete with full control paths.

'''

import time

import serial
from Src.robot.arm.rbt_constants import *


# A class that stores information about the stepper motor.
class StepperHandler:

    def __init__(self):
        self.stepper_port = STEPPER_PORT
        self.baud_rate = STEPPER_BAUD
        self.stepper_serial = None

        print(f"[RAIL] Connecting to serial port: {self.stepper_port}, baud rate:{self.baud_rate}")
        self.stepper_serial = serial.Serial(self.stepper_port, self.baud_rate)

        self.initialized = False
        self.initialize_motor()

    def initialize_motor(self):
        self.stepper_serial.write(b'\r\n\r\n')
        time.sleep(1)
        self.stepper_serial.flushInput()
        print(f"[RAIL] Stepper motor enabled!")
        print("[RAIL] Current position is home.")
        self.initialized = True

    def write_x(self, pos, feed=100):
        code = bytes(str(f"G1 X{round(pos[2])} F{round(feed)}"), "ASCII")
        self.stepper_serial.flushInput()
        self.stepper_serial.write(code + b'\r\n')
        print(code)

    def write_y(self, pos, feed=100):
        code = bytes(str(f"G1 Y{round(pos[2])} F{round(feed)}"), "ASCII")
        self.stepper_serial.flushInput()
        self.stepper_serial.write(code + b'\r\n')
        print(code)

    def write_z(self, pos, feed=100):
        code = bytes(str(f"G1 Z{round(pos[2])} F{round(feed)}"), "ASCII")
        self.stepper_serial.flushInput()
        self.stepper_serial.write(code + b'\r\n')
        print(code)

    def write_a(self, pos, feed=100):
        code = bytes(str(f"G1 A{round(pos[2])} F{round(feed)}"), "ASCII")
        self.stepper_serial.flushInput()
        self.stepper_serial.write(code + b'\r\n')
        print(code)

    def write_b(self, pos, feed=100):
        code = bytes(str(f"G1 B{round(pos[2])} F{round(feed)}"), "ASCII")
        self.stepper_serial.flushInput()
        self.stepper_serial.write(code + b'\r\n')
        print(code)

    def close(self):
        print(f"[RAIL] Exiting... going home.")
        print("[RAIL] I did nothing... please implement me :]")
        self.stepper_serial.close()

def generate_g_code(axis, goal_position, feed_rate):
    rot_code = bytes(str(goal_position), "ASCII")
    feed_code = bytes(str(feed_rate), "ASCII")
    axis = bytes(str(axis), "ASCII")
    return b'G1 ' + axis + rot_code + b' F' + feed_code + b'\r\n'
