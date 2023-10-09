import time

import numpy as np
from Src.robot.SerialMonitor import LDS
from Src.robot.SerialMonitor import *


# possibly useful article if 3D board does NOT repsond 'OK' after a move is completed:
# https://forum.duet3d.com/topic/18282/tighter-control-for-waiting-for-motion-commands-to-complete/2
class Scanner:

    def __init__(self, stepper_board, port_conf, obj_x, obj_y, obj_z, run_thread=None, percentage_widget=None):
        self.LDS = LDS(port_conf.lds_port, port_conf.lds_baud)
        self.stepper_board = stepper_board

        self.percentage_widget = percentage_widget
        self.run_thread = run_thread

        self.xc = np.array([])
        self.yc = np.array([])
        self.zc = np.array([])
        self.ca = np.array([])

        self.x_start = 0
        self.z_start = 0
        self.x_end = 203
        self.z_end = 203

        self.x_feed = 1500
        self.z_feed = 500

        self.step = 1

        self.degree = 45
        self.rotations = 360 / self.degree

        # add some statistics for viewing
        self.total_points = ((self.x_end - self.x_start) * (self.z_end - self.z_start)) * self.rotations
        self.point_index = 0
        self.stop = False

    def begin_scan(self):
        for h in range(0, int(self.rotations)):
            cur_axis = h * self.degree
            print("Current axis: ", cur_axis)
            for j in range(self.z_start, self.z_end):
                z_pos = j

                for i in range(self.x_start, self.x_end):
                    if self.stop:
                        self.LDS.laser.close()
                        return

                    x_pos = i

                    if x_pos == self.x_end - 1:
                        time.sleep(1)
                        self.stepper_board.write_x(self.x_start, self.x_feed)
                        self.stepper_board.read_data()  # wait for response
                        time.sleep(5)
                    else:
                        self.stepper_board.write_x(x_pos + self.step, self.x_feed)
                        self.stepper_board.read_data()  # wait for response or wait a fixed amount of time if no work
                        # time.sleep(0.0001)
                        # time.sleep(0.05) # ensure that the distance is read before the next move, might need to adjust or maybe we can get rid of entirely
                        xc = np.append(self.xc, x_pos)
                        zc = np.append(self.zc, z_pos)
                        yc = np.append(self.yc,
                                       self.LDS.read_distance())  # change this to the adjusted read in value from the sensor
                        ca = np.append(self.ca, cur_axis)
                        self.point_index += 1

                        if self.percentage_widget:
                            self.percentage_widget.setValue(int((self.point_index / self.total_points) * 100))

                        print("Saved point: ", xc[-1], zc[-1], yc[-1], " degree: ", ca[-1])

                if z_pos == self.z_end - 1:
                    self.stepper_board.write_z(self.z_start, self.z_feed)
                    self.stepper_board.read_data()  # wait for response
                    time.sleep(5)
                else:
                    self.stepper_board.write_z(z_pos + self.step, self.z_feed)
                    self.stepper_board.read_data()  # wait for response
                    time.sleep(5)

            self.stepper_board.write_a(cur_axis, F=1500)
            time.sleep(10)


class PrimitiveScan:

    def __init__(self, stepper_board: StepperHandler, port_conf):
        self.LDS = LDS(port_conf.lds_port, port_conf.lds_baud)
        self.stepper_board = stepper_board

        self.xc = np.array([])
        self.yc = np.array([])
        self.zc = np.array([])
        self.ca = np.array([])

        self.x_feed = 1500
        self.x_start = 0
        self.z_feed = 500
        self.z_start = 10
        self.a_feed = 500
        self.a_start = 0

        self.step = 1

        self.degree = 45
        self.rotations = 360 / self.degree

    def run_prim_scan(self, primitive, x, y, z):
        val = x
        val_2 = y
        self.z_start = z / 4
        if primitive == 2:
            self.edge_1 = int(101.5 - val / 2)  # Take data from inputted face and /2 to put you a little edge
            self.edge_2 = int(101.5 + val / 2)  # Same as above but other direction
            self.edge_3 = int(101.5 - val_2 / 2)  # Ditto
            self.edge_4 = int(101.5 + val_2 / 2)  # Ditto
            self.offset_1 = 0  # Read in value of LDS and subtract offset based off center for the first side
            self.offset_2 = 0  # Read in value of LDS and subtract offset based off center for the second side
            self.edge_actual_1 = 0
            self.edge_actual_2 = 0
            for h in range(0, 2):  # Just measure two sides
                self.stepper_board.write_a(h * 90, feed=1800)
                self.stepper_board.write_z(self.z_start, self.z_feed)
                self.stepper_board.write_x(self.edge_1 - 5, self.x_feed)

                for i in range(self.edge_1 - 5, self.edge_2 + 5):
                    x_pos = i
                    y = 17500 - (self.LDS.read_distance() + 10000)
                    if (y < 17500 and y > -3000) and self.edge_actual_1 == 0:
                        self.edge_actual_1 = x_pos
                        self.offset_1 = val - self.edge_actual_1
                        print("offset 1: ", self.offset_1)
                    elif (y > 17500 or y < -3000) and self.edge_actual_2 == 0 and self.offset_1 != 0:
                        self.edge_actual_2 = x_pos
                        self.offset_2 = val - self.edge_actual_2
                        print("offset 2: ", self.offset_2)
                    if x_pos == self.edge_2 + 4:
                        self.stepper_board.write_x(self.edge_1, self.x_feed)
                        time.sleep(5)  # Wait for motor to reset to beginning of loop
                    else:
                        self.stepper_board.write_x(x_pos + self.step, self.x_feed)
                        self.stepper_board.read_data()  # change this to just longer than feed rate
            self.stepper_board.write_a(0, feed=1800)
            self.stepper_board.home_scan()
            time.sleep(5)
        elif primitive == 1: # Cylinder
            self.radius_1 = 101.5 - val / 2
            self.radius_2 = 101.5 + val / 2
            self.offset_1 = 0  # Read in value of LDS and subtract offset based off center for the first side
            self.offset_2 = 0  # Read in value of LDS and subtract offset based off center for the second side
            self.radius_actual_1 = 0
            self.radius_actual_2 = 0

            for h in range(0, 2):  # Just measure two sides
                self.stepper_board.write_a(90, feed=1800)
                self.stepper_board.write_z(self.z_start, self.z_feed)
                self.stepper_board.write_x(self.radius_1 - 5, self.x_feed)

                for i in range(self.radius_1 - 5, self.radius_2 + 5):
                    x_pos = i
                    self.xc = np.append(self.xc, x_pos)
                    self.yc = np.append(self.yc,
                                        self.LDS.read_distance())  # change this to the adjusted read in value from the sensor

                    if h == 0 and i >= self.radius_2 + 5:
                        self.radius_actual_1 = min(self.yc)
                        self.offset_1 = self.radius_1 - self.radius_actual_1
                        self.yc = np.array([])
                    else:
                        self.radius_actual_2 = min(self.yc)
                        self.offset_2 = self.radius_2 - self.radius_actual_2

                    if x_pos == self.radius_2 + 4:
                        time.sleep(1)
                        self.stepper_board.write_x(self.radius_1, self.x_feed)
                        self.stepper_board.read_data()  # wait for response
                        time.sleep(5)  # Wait for motor to reset to beginning of loop
                    else:
                        self.stepper_board.write_x(x_pos + self.step, self.x_feed)
                        self.stepper_board.read_data()  # change this to just longer than feed rate
            self.stepper_board.write_a(90, feed=1800)
            self.LDS.laser.close()
            time.sleep(5)

        elif primitive == 2:
            self.radius = val
            self.radius_1 = 101.5 - val / 2
            self.radius_2 = 101.5 + val / 2
            self.offset_1 = 0  # Read in value of LDS and subtract offset based off center for the first side
            self.offset_2 = 0  # Read in value of LDS and subtract offset based off center for the second side
            self.radius_actual_1 = 0
            self.radius_actual_2 = 0
            for h in range(0, 2):  # Just measure two sides
                self.stepper_board.write_a(90 * 35.5, feed=1800)
                self.stepper_board.write_z(self.radius, self.z_feed)
                self.stepper_board.write_x(self.radius_1 - 5, self.x_feed)

                for i in range(self.radius_1 - 5, self.radius_2 + 5):
                    x_pos = i
                    self.xc = np.append(self.xc, x_pos)
                    self.yc = np.append(self.yc, self.LDS.read_distance())
                    if h == 0 and i >= self.radius_2:
                        self.radius_actual_1 = min(self.yc)
                        self.offset_1 = self.radius_1 - self.radius_actual_1
                        self.yc = np.array([])
                    else:
                        self.radius_actual_2 = min(self.yc)
                        self.offset_2 = self.radius_2 - self.radius_actual_2
                    if x_pos == self.radius_2 + 4:
                        time.sleep(1)
                        self.stepper_board.write_x(self.radius_1, self.x_feed)
                        self.stepper_board.read_data()  # wait for response
                        time.sleep(5)  # Wait for motor to reset to beginning of loop
                    else:
                        self.stepper_board.write_x(x_pos + self.step, self.x_feed)
                        self.stepper_board.read_data()  # change this to just longer than feed rate
                self.stepper_board.write_a(90, feed=1800)
                time.sleep(5)
        else:
            print("Primitive Object scan was not properly called")


import numpy as np
import time

from serial import Serial, PARITY_NONE, EIGHTBITS
from Src.robot.SerialMonitor import LDS, StepperHandler, SerialMonitor


class Discharge:

    def __init__(self, obj_pos, stepper_board=None):

        self.LDS = LDS()
        self.stepper_board = stepper_board

        # def scanner_points(self):
        self.x_feed = 1500
        self.y_feed = 500  # This needs to be changed to the equivalent of 50mm/s
        self.z_feed = 500
        self.step = 1

        # get z of object
        self.obj_z = obj_pos[2]

    def set_stepper(self, stepper):
        self.stepper_board = stepper

    def discharge(self):
        if not self.stepper_board:
            print("error did not set stepepr board.")

        self.stepper_board.home_scan()

        self.stepper_board.write_z((self.obj_z / 2) - 25, self.z_feed)
        self.stepper_board.write_x(102, self.x_feed)
        time.sleep(10)

        y = 17500 - (self.LDS.read_distance() + 10000)

        # write to z to half object height (-25 to account for CVR)
        # write to x to center (102)

        print("writing: ", y)

        self.stepper_board.write_y((y-30), self.y_feed)
        self.stepper_board.read_data()
        time.sleep(10)

        self.stepper_board.home_scan()
        self.LDS.laser.close()

