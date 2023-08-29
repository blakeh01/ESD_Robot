import time

import serial
from src.robot.rbt_constants import *


# A class that stores information about the stepper motor.
class StepperHandler:

    def __init__(self):
        self.stepper_port = STEPPER_PORT
        self.baud_rate = STEPPER_BAUD
        self.stepper_serial = None

        print(f"[RAIL] Connecting to serial port: {self.stepper_port}, baud rate:{self.baud_rate}")
        self.stepper_serial = serial.Serial(self.stepper_port, self.baud_rate)

        self.arrived = False

        self.home_pos = 0
        self.current_pos = 0
        self.max_pos = 2270
        self.goal_pos = 0

        self.feed_rate = 15000

        self.spin_amt = 0

        self.initialized = False
        self.initialize_motor()

    def initialize_motor(self):
        self.stepper_serial.write(b'\r\n\r\n')
        time.sleep(1)
        self.stepper_serial.flushInput()
        print(f"[RAIL] Stepper motor enabled!")
        print("[RAIL] Current position is home.")
        self.initialized = True

    def update_motor(self, t=0):
        # possible fix for multiple write commands:
        # set goal positon when writing position
        # if goal position != current positon
        # increment the current position toward to goal position by some step amount
        # continue doing that until it reaches goal position.
        pass

    def write_position(self, rot):
        return
        if rot > self.max_pos or rot < 0 or rot == self.goal_pos:  # Ensure we do not write an unreachable rotation
            return

        self.current_pos = rot
        self.stepper_serial.write(self.generate_g_code(rot))

    def write_3axis(self, pos):
        code = bytes(str(f"G1 X{round(pos[0])} Y{round(pos[1])} Z{round(pos[2])} F250"), "ASCII")
        self.stepper_serial.write(code + b'\r\n')
        # print(code)

    def spin_plat(self):
        self.spin_amt += 4.1
        code = bytes(str(f"G1 A{self.spin_amt} F25"), "ASCII")
        self.stepper_serial.write(code + b'\r\n')

    def generate_g_code(self, rot):
        rot_code = bytes(str(rot), "ASCII")
        feed_code = bytes(str(self.feed_rate), "ASCII")
        return b'G1 Y' + rot_code + b' F' + feed_code + b'\r\n'

    def close(self):
        print(f"[RAIL] Exiting... going home.")
        self.stepper_serial.write(self.generate_g_code(self.home_pos))
        self.stepper_serial.write(bytes(str(f"G1 X0 Y0 Z0 F250"), "ASCII"))
        self.stepper_serial.close()
