import os
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from src.robot.SerialMonitor import *
from datetime import datetime


# possibly useful article if 3D board does NOT respond 'OK' after a move is completed:
# https://forum.duet3d.com/topic/18282/tighter-control-for-waiting-for-motion-commands-to-complete/2
class ObjectScanner:

    def __init__(self, stepper_board: StepperHandler, lds: LDS, obj_x, obj_y, obj_z, run_thread=None,
                 percentage_widget=None):
        self.stepper_board = stepper_board
        self.lds = lds

        self.percentage_widget = percentage_widget
        self.run_thread = run_thread

        self.data_arr = []  # stores x,y,z of scanned point as well as the current platform rotation for reconstruction.

        self.x_start = 0
        self.z_start = 0
        self.x_end = 203  # in mm
        self.z_end = 203  # in mm

        self.x_feed = 1500  # in mm/min
        self.z_feed = 500   # in mm/min
        self.rot_feed = 1600    # in mm/min

        self.degree = 45
        self.rotations = 360 / self.degree

        # add some statistics for viewing
        self.total_points = ((self.x_end - self.x_start) * (self.z_end - self.z_start)) * self.rotations
        self.point_index = 0
        self.stop = False

        self.file_path = f"scanned-points_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def start_scan_interp(self):
        """
        Uses interpolation of stepper position to more quickly scan an object at a cost of accuracy.

        This algorithm works by calculating how long it takes for the X axis rail to complete the self.x_start
        to self.x_end movement based on self.x_feed. It then divides this time by the number of points needing
        to be scanned to get a 'scanning rate' by sleeping at this interval, we can expect the laser to report a
        measurement at each mm.

        In other words, time system will sleep for 1 mm, scan, then repeat. Where 1 mm travel time is simply dt.
        """
        # create excel to write points to
        df = pd.DataFrame(self.data_arr, columns=['Rotation', 'X-Pos', 'Y-Pos', 'Z-Pos'])
        df.to_excel(self.file_path, index=False, sheet_name='ObjectData')

        # rotate object 45 degrees 7 times (0-315, covering entire object)
        for h in range(0, int(self.rotations)):
            rot = h * self.degree

            # iterate through z_start to z_end
            for z_pos in range(self.z_start, self.z_end):
                self.stepper_board.write_z(z_pos, self.z_feed)
                time.sleep(1)

                steps = range(self.x_start, self.x_end)

                # calculate how long it takes to reach x_end going x_feed mm/min, then divide by number of steps
                # to get the interval at which the scanner should scan.
                dt = ((self.x_end - self.x_start) / (self.x_feed/60)) / len(steps)

                self.stepper_board.write_x(self.x_end, self.x_feed)  # begin motion to end of rail
                for x_pos in steps:  # go through each step, sleeping for dt so scanning only on proper interval.
                    dist_mm = self.lds.get_absolute_distance() / 100  # / 100 to get into mm.

                    # read LDS and append position/rot to the data arr. ignore points that are 'maxed' out.
                    if dist_mm < 420:
                        self.data_arr.append([rot, x_pos, dist_mm, z_pos])

                    self.point_index += 1
                    if self.percentage_widget:
                        self.percentage_widget.setValue(int((self.point_index / self.total_points) * 100))

                    print("Saved point: ", x_pos, dist_mm, z_pos, " degree: ", rot)

                    time.sleep(dt)  # sleep interval time.

                self.stepper_board.write_x(self.x_start, self.x_feed)  # move x rail back to start
                time.sleep(8.5)

                print("Saving points for this z-slice...")

                df = pd.DataFrame(self.data_arr, columns=['rot', 'x', 'y', 'z'])  # create pandas df of data
                # Append to the existing file
                with pd.ExcelWriter(self.file_path, engine='openpyxl', mode='a') as writer:
                    df.to_excel(writer, index=False, sheet_name='ObjectData', header=False)

                self.data_arr = []  # clear array after writing

            self.stepper_board.write_rot_platform(rot, self.rot_feed)
            self.stepper_board.write_xyz(0, 0, 0)
            time.sleep(10)

        return True

    def start_scan(self):
        """
        Waits a static amount of time between each stepper movement to ensure the stepper position is known. Highest
        accuracy, but slowest performance.
        """
        # create excel to write points to
        df = pd.DataFrame(self.data_arr, columns=['Rotation', 'X-Pos', 'Y-Pos', 'Z-Pos'])
        df.to_excel(self.file_path, index=False, sheet_name='ObjectData')

        # rotate object 45 degrees 7 times (0-315, covering entire object)
        for h in range(0, int(self.rotations)):
            rot = h * self.degree

            # iterate through z_start to z_end
            for z_pos in np.arange(self.z_start, self.z_end, step=1):
                self.stepper_board.write_z(z_pos, self.z_feed)
                time.sleep(0.5)

                for x_pos in np.arange(self.x_start, self.x_end, step=1):

                    if self.stop:  # allows the for loop to be interrupted if halt is called
                        return False

                    self.stepper_board.write_x(x_pos, self.x_feed)
                    time.sleep(0.2)

                    dist_mm = self.lds.get_absolute_distance() / 100  # / 100 to get into mm.

                    # read LDS and append position/rot to the data arr. ignore points that are 'maxed' out.
                    if dist_mm < 420:
                        self.data_arr.append([rot, x_pos, dist_mm, z_pos])

                    self.point_index += 1
                    if self.percentage_widget:
                        self.percentage_widget.setValue(int((self.point_index / self.total_points) * 100))

                    print("Saved point: ", x_pos, dist_mm, z_pos, " degree: ", rot)

                self.stepper_board.write_x(self.x_start, self.x_feed)  # move x rail back to start
                time.sleep(8.5)

                print("Saving points for this z-slice...")

                df = pd.DataFrame(self.data_arr, columns=['rot', 'x', 'y', 'z'])  # create pandas df of data
                # Append to the existing file
                with pd.ExcelWriter(self.file_path, engine='openpyxl', mode='a') as writer:
                    df.to_excel(writer, index=False, sheet_name='ObjectData', header=False)

                self.data_arr = []  # clear array after writing

            self.stepper_board.write_rot_platform(rot, self.rot_feed)
            self.stepper_board.write_xyz(0, 0, 0)
            time.sleep(7.5)

        return True


X_PLAT_MIDDLE = 101  # corresponds to middle of platform when x pos is set to 101
X_PLAT_MAX = 202  # corresponds to right most (from perspective of laser) x position

LDS_MAX_RANGE = 42766  # LDS returns this value when the laser cannot read the distance (either too close or too far)


class EdgeFinder:

    def __init__(self, stepper_board: StepperHandler, lds: LDS):
        self.stepper_board = stepper_board
        self.lds = lds

        self.x_feed = 1500
        self.y_feed = 500
        self.z_feed = 500
        self.a_feed = 3600

        self.step = 1

        self.degree = 45
        self.rotations = 360 / self.degree

    def find_object_edges(self, primitive, *obj_data):
        # set XYZ and rot positions to 0
        self.stepper_board.write_xyz(0, 0, 0)
        self.stepper_board.write_rot_platform(0, self.a_feed)

        time.sleep(5)

        if primitive == "Rectangular Prism":
            # get object parameters
            x, y, z = obj_data[0], obj_data[1], obj_data[2]
            print("Finding rect edges... expected edges at: (x) ", (X_PLAT_MIDDLE - x / 2), " and ",
                  (X_PLAT_MIDDLE - x / 2), ", and (y), ", (X_PLAT_MIDDLE - y / 2), " and ", (X_PLAT_MIDDLE - y / 2))
            ####### (from perspective of laser) find left most edge #######

            y = self.lds.get_absolute_distance()  # get current distance
            left_edge_x = 0  # coordinate of left edge, shifted 101
            while y >= LDS_MAX_RANGE:  # while LDS is out of range (no edge detected)
                # move stepper board by 1 mm, wait, then check laser again to see if edge was detected
                self.stepper_board.write_x(left_edge_x, self.x_feed)
                left_edge_x += 1
                time.sleep(0.1)

                y = self.lds.get_absolute_distance()

            print("X Left edge found @ ", left_edge_x)

            ####### (from perspective of laser) find right most edge #######

            self.stepper_board.write_x(X_PLAT_MIDDLE, self.x_feed)  # set laser to middle of platform, wait
            time.sleep(5)

            y = self.lds.get_absolute_distance()
            right_edge_x = X_PLAT_MIDDLE
            while y <= LDS_MAX_RANGE:  # while the LDS is in range (on the object)
                # move stepper board by 1 mm, wait, then check laser again to see if edge was detected
                self.stepper_board.write_x(right_edge_x, self.x_feed * 2)
                right_edge_x += 1
                time.sleep(0.2)

                y = self.lds.get_absolute_distance()

            print("X Right edge found @ ", right_edge_x)

            ####### ROTATE PLATFORM, MEASURING ALONG Y NOW... ########
            self.stepper_board.write_rot_platform(90, self.a_feed)
            self.stepper_board.write_x(0, self.x_feed)  # reset x position
            time.sleep(10)

            ####### (from perspective of laser) find left most edge #######

            y = self.lds.get_absolute_distance()  # get current distance
            left_edge_x = 0  # coordinate of left edge, shifted 101
            while y >= LDS_MAX_RANGE:  # while LDS is out of range (no edge detected)
                # move stepper board by 1 mm, wait, then check laser again to see if edge was detected
                self.stepper_board.write_x(left_edge_x, self.x_feed * 2)
                left_edge_x += 1
                time.sleep(0.2)

                y = self.lds.get_absolute_distance()

            print("Y Left edge found @ ", left_edge_x)

            ####### (from perspective of laser) find right most edge #######

            self.stepper_board.write_x(X_PLAT_MIDDLE, self.x_feed)  # set laser to middle of platform, wait
            time.sleep(5)

            y = self.lds.get_absolute_distance()
            right_edge_x = X_PLAT_MIDDLE
            while y <= LDS_MAX_RANGE:  # while the LDS is in range (on the object)
                # move stepper board by 1 mm, wait, then check laser again to see if edge was detected
                self.stepper_board.write_x(right_edge_x, self.x_feed)
                right_edge_x += 1
                time.sleep(0.1)

                y = self.lds.get_absolute_distance()

            print("Y Right edge found @ ", right_edge_x)

            self.stepper_board.write_xyz(0, 0, 0)
            self.stepper_board.write_rot_platform(0, feed=self.a_feed)

        elif primitive == "Cylinder":
            # get object parameters
            r, h = obj_data[0], obj_data[1]
            print("Finding cylinder edges... expected edges at: ", (X_PLAT_MIDDLE - r))

            ####### (from perspective of laser) move left until off of the object #######
            self.stepper_board.write_xyz(X_PLAT_MIDDLE, 0, h / 2 - 15)  # minus 15 for LDS offset
            time.sleep(10)

            y = self.lds.get_absolute_distance()
            f_radius = X_PLAT_MIDDLE
            while y <= LDS_MAX_RANGE:  # while the LDS is in range (on the object)
                # move stepper board by 1 mm, wait, then check laser again to see if edge was detected
                self.stepper_board.write_x(f_radius, self.x_feed)
                f_radius -= 1
                time.sleep(0.15)

                y = self.lds.get_absolute_distance()

            x_off = round(f_radius - (X_PLAT_MIDDLE - r))
            print("Radius found: ", f_radius)
            print("Calculated offset on X: ", x_off)

            ####### ROTATE PLATFORM, MEASURING ALONG Y NOW... ########
            self.stepper_board.write_rot_platform(90, self.a_feed)
            self.stepper_board.write_x(X_PLAT_MIDDLE, self.x_feed)  # set laser to middle of platform, wait
            time.sleep(10)

            y = self.lds.get_absolute_distance()
            f_radius = X_PLAT_MIDDLE
            while y <= LDS_MAX_RANGE:  # while the LDS is in range (on the object)
                # move stepper board by 1 mm, wait, then check laser again to see if edge was detected
                self.stepper_board.write_x(f_radius, self.x_feed)
                f_radius -= 1
                time.sleep(0.15)

                y = self.lds.get_absolute_distance()

            y_off = round(f_radius - (X_PLAT_MIDDLE - r))  # 1 mm acc
            print("Radius found: ", f_radius)
            print("Calculated offset on Y: ", y_off)

            time.sleep(0.5)

            self.stepper_board.write_xyz(0, 0, 0)
            self.stepper_board.write_rot_platform(0, feed=self.a_feed)
            return x_off, y_off

        elif primitive == "Sphere":
            pass
