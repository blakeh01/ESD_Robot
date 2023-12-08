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

    def __init__(self, stepper_board: StepperHandler, lds: LDS, obj_x, obj_y, obj_z, run_thread=None, percentage_widget=None):
        self.stepper_board = stepper_board
        self.lds = lds

        self.percentage_widget = percentage_widget
        self.run_thread = run_thread

        self.data_arr = []  # stores x,y,z of scanned point as well as the current platform rotation for reconstruction.

        self.x_start = 0
        self.z_start = 0
        self.x_end = 203
        self.z_end = 203

        self.x_feed = 1500
        self.z_feed = 500
        self.rot_feed = 1600

        self.step = 1

        self.degree = 45
        self.rotations = 360 / self.degree

        # add some statistics for viewing
        self.total_points = ((self.x_end - self.x_start) * (self.z_end - self.z_start)) * self.rotations
        self.point_index = 0
        self.stop = False

        self.file_path = f"scanned-points_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def start_scan(self):
        # create excel to write points to
        df = pd.DataFrame(self.data_arr, columns=['Rotation', 'X-Pos', 'Y-Pos', 'Z-Pos'])
        df.to_excel(self.file_path, index=False, sheet_name='ObjectData')

        # create a 3D plot to view points on the current slice:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # rotate object 45 degrees 7 times (0-315, covering entire object)
        for h in range(0, int(self.rotations)):
            rot = h * self.degree

            # iterate through z_start to z_end
            for z_pos in range(self.z_start, self.z_end):
                self.stepper_board.write_z(z_pos, self.z_feed)
                self.stepper_board.read_data()  # a way to wait for response from stepper before continuing

                for x_pos in range(self.x_start, self.x_end):

                    if self.stop:  # allows the for loop to be interrupted if halt is called
                        return

                    self.stepper_board.write_x(x_pos, self.x_feed)
                    self.stepper_board.read_data()  # a way to wait for response from stepper before reading data

                    # read LDS and append position/rot to the data arr
                    self.data_arr.append([rot, x_pos, self.lds.get_absolute_distance(), z_pos])
                    self.point_index += 1

                    if self.percentage_widget:
                        self.percentage_widget.setValue(int((self.point_index / self.total_points) * 100))

                    print("Saved point: ", x_pos, self.lds.get_absolute_distance(), z_pos, " degree: ", rot)

                self.stepper_board.write_x(self.x_start, self.x_feed)  # move x rail back to start
                self.stepper_board.read_data()  # wait for response

                print("Saving points for this z-slice...")

                df = pd.DataFrame(self.data_arr, columns=['rot', 'x', 'y', 'z'])  # create pandas df of data
                self.update_live_plot(ax, df)  # update plot

                # Append to the existing file
                with pd.ExcelWriter(self.file_path, engine='openpyxl', mode='a') as writer:
                    df.to_excel(writer, index=False, sheet_name='ObjectData', header=False)

                self.data_arr = []  # clear array after writing

            self.stepper_board.write_rot_platform(rot, self.rot_feed)
            self.stepper_board.read_data()
            # time.sleep(2.5)

    def update_live_plot(self, ax, df):
        # Update the 3D scatter plot for the current slice along the x-axis
        ax.clear()
        ax.scatter(df['x'], df['y'], df['z'], c=df['rot'], cmap='viridis')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()


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
            print("Finding rect edges... expected edges at: (x) ", (X_PLAT_MIDDLE - x/2), " and ", (X_PLAT_MIDDLE - x/2), ", and (y), ", (X_PLAT_MIDDLE - y/2), " and ", (X_PLAT_MIDDLE - y/2))
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
                self.stepper_board.write_x(right_edge_x, self.x_feed*2)
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
                self.stepper_board.write_x(left_edge_x, self.x_feed*2)
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
            self.stepper_board.write_xyz(X_PLAT_MIDDLE, 0, h/2 - 15)  # minus 15 for LDS offset
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
