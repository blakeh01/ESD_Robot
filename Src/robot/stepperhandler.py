'''

    This class needs work.

    StepperHandler keeps track of the linear rail (and maybe 3d axis system) and sends gcode to update position

    The current interpolator is broken, and does not function as intended.

    The position needs to be interpolated as the 3d printer controller does NOT report back the current
    stepper position.

    Also... Gcode is meant to be preprocessed and execute in parallel with the controls.
    Which is the ultimate goal, so this class might be obsolete with full control paths.

'''

import serial
import time

from Src.robot.rbt_constants import *


# A class that stores information about the stepper motor.
class StepperHandler:

    def __init__(self):
        self.stepper_port = STEPPER_PORT
        self.baud_rate = STEPPER_BAUD
        self.stepper_serial = None

        print(f"[RAIL] Connecting to serial port: {self.stepper_port}, baud rate:{self.baud_rate}")
        self.stepper_serial = serial.Serial(self.stepper_port, self.baud_rate)

        print(f"[RAIL] Creating robot linear rail instance...")
        self.linear_rail_motor = LinearRail()

        self.initialized = False
        self.initialize_motor()

    def initialize_motor(self):
        self.stepper_serial.write(b'\r\n\r\n')
        time.sleep(1)
        self.stepper_serial.flushInput()
        print(f"[RAIL] Stepper motor enabled!")
        print("[RAIL] Current position is home.")
        self.initialized = True

    def update_interpolated_positions(self, dt):
        self.linear_rail_motor.update(dt)

    def write_3axis(self, pos):
        code = bytes(str(f"G1 X{round(pos[0])} Y{round(pos[1])} Z{round(pos[2])} F500"), "ASCII")
        self.stepper_serial.flushInput()
        self.stepper_serial.write(code + b'\r\n')
        print(code)

    def close(self):
        print(f"[RAIL] Exiting... going home.")
        self.stepper_serial.write(generate_g_code('A', 0, 500))
        self.stepper_serial.close()


class LinearRail:

    def __init__(self):
        self.feed_rate = 20
        self.position = 0
        self.target_position = None

        self.position_queue = []

    def add_position_to_queue(self, target_position):
        self.position_queue.append(target_position)

    def update(self, dt):
        if self.target_position is None and len(self.position_queue) > 0:
            self.target_position = self.position_queue.pop()
            # send g code

        if self.target_position is not None:
            distance = self.feed_rate * dt
            if self.position < self.target_position:
                self.position = min(self.position + distance, self.target_position)
            else:
                self.position = max(self.position - distance, self.target_position)

            if self.position == self.target_position:
                self.target_position = None
                print("Movement Completed")
            # else:
            #     print(f"Movement Progress: {self.position:.2f}")

    def get_position(self):
        return self.position

    def set_feed_rate(self, feed_rate):
        self.feed_rate = feed_rate


def generate_g_code(axis, goal_position, feed_rate):
    rot_code = bytes(str(goal_position), "ASCII")
    feed_code = bytes(str(feed_rate), "ASCII")
    axis = bytes(str(axis), "ASCII")
    return b'G1 ' + axis + rot_code + b' F' + feed_code + b'\r\n'
