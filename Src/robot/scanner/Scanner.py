import numpy as np
import time

from serial import Serial, PARITY_NONE, EIGHTBITS
from Src.robot.SerialMonitor import LDS, StepperHandler, SerialMonitor


# possibly useful article if 3D board does NOT repsond 'OK' after a move is completed:
# https://forum.duet3d.com/topic/18282/tighter-control-for-waiting-for-motion-commands-to-complete/2
class Scanner:

    def __init__(self, stepper_board):
        self.LDS = LDS()
        self.stepper_board = stepper_board

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

    def begin_scan(self):
        for h in range(0, int(self.rotations)):
            cur_axis = h * self.degree
            print("Current axis: ", cur_axis)
            for j in range(self.z_start, self.z_end):
                z_pos = j

                for i in range(self.x_start, self.x_end):
                    x_pos = i

                    if x_pos == self.x_end - 1:
                        time.sleep(1)
                        self.stepper_board.write_x(self.x_start, self.x_feed)
                        self.stepper_board.read_data() # wait for response
                        time.sleep(5)
                    else:
                        self.stepper_board.write_x(x_pos + self.step, self.x_feed)
                        self.stepper_board.read_data() # wait for response or wait a fixed amount of time if no work
                        #time.sleep(0.0001)
                        #time.sleep(0.05) # ensure that the distance is read before the next move, might need to adjust or maybe we can get rid of entirely
                        xc = np.append(self.xc, x_pos)
                        zc = np.append(self.zc, z_pos)
                        yc = np.append(self.yc, self.LDS.read_distance()) # change this to the adjusted read in value from the sensor
                        ca = np.append(self.ca, cur_axis)

                        print("Saved point: ", xc[-1], zc[-1], yc[-1], " degree: ", ca[-1])


                if z_pos == self.z_end - 1:
                    self.stepper_board.write_z(self.z_start, self.z_feed)
                    self.stepper_board.read_data()  # wait for response
                    time.sleep(5)
                else:
                    self.stepper_board.write_z(z_pos + self.step, self.z_feed)
                    self.stepper_board.read_data()  # wait for response
                    time.sleep(5)

            self.stepper_board.write_a(cur_axis * 35.5, F=1800)
            time.sleep(10)