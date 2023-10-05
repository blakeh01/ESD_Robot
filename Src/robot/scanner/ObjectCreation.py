'''

This file will read in the points from the pathing loop and figure out and delete any similar points

'''

import numpy as np
import open3d as o3d

from Src.robot.SerialMonitor import LDS


class ScannedObjectCreation:
    def __init__(self, stepper_board):

        self.LDS = LDS()
        self.stepper_board = stepper_board

        self.xc = np.array([])
        self.yc = np.array([])
        self.zc = np.array([])
        self.ca = np.array([])
        self.degree = 45

        # Testing points (Comment these out or delete)
        # self.xc = np.array([0, 5, 10, 0, 5, 10, 0, 5, 10, 10])
        # self.yc = np.array([0, 5, 10, 10, 10, 30, 30, 30, 40, 40])
        # self.zc = np.array([0, 25, 25, 25, 25, 25, 25, 25, 25, 25])
        # self.ca = np.array([0, 1, 2, 3, 4, 5, 6, 7, 7, 0])

        # Initialize the arrays
        self.xr1 = np.array([]);
        self.yr1 = np.array([]);
        self.zr1 = np.array([])
        self.xr2 = np.array([]);
        self.yr2 = np.array([]);
        self.zr2 = np.array([])
        self.xr3 = np.array([]);
        self.yr3 = np.array([]);
        self.zr3 = np.array([])
        self.xr4 = np.array([]);
        self.yr4 = np.array([]);
        self.zr4 = np.array([])
        self.xr5 = np.array([]);
        self.yr5 = np.array([]);
        self.zr5 = np.array([])
        self.xr6 = np.array([]);
        self.yr6 = np.array([]);
        self.zr6 = np.array([])
        self.xr7 = np.array([]);
        self.yr7 = np.array([]);
        self.zr7 = np.array([])
        self.xr8 = np.array([]);
        self.yr8 = np.array([]);
        self.zr8 = np.array([])

        # Categorize the different scanned points into their correct scanned angle
        for a in range(0, (np.size(self.ca))):
            if self.ca[a] == 0:
                self.xr1 = np.append(self.xr1, self.xc[a])
                self.yr1 = np.append(self.yr1, self.yc[a])
                self.zr1 = np.append(self.zr1, self.zc[a])
            elif self.ca[a] == 1:
                self.xr2 = np.append(self.xr2, self.xc[a])
                self.yr2 = np.append(self.yr2, self.yc[a])
                self.zr2 = np.append(self.zr2, self.zc[a])
            elif self.ca[a] == 2:
                self.xr3 = np.append(self.xr3, self.xc[a])
                self.yr3 = np.append(self.yr3, self.yc[a])
                self.zr3 = np.append(self.zr3, self.zc[a])
            elif self.ca[a] == 3:
                self.xr4 = np.append(self.xr4, self.xc[a])
                self.yr4 = np.append(self.yr4, self.yc[a])
                self.zr4 = np.append(self.zr4, self.zc[a])
            elif self.ca[a] == 4:
                self.xr5 = np.append(self.xr5, self.xc[a])
                self.yr5 = np.append(self.yr5, self.yc[a])
                self.zr5 = np.append(self.zr5, self.zc[a])
            elif self.ca[a] == 5:
                self.xr6 = np.append(self.xr6, self.xc[a])
                self.yr6 = np.append(self.yr6, self.yc[a])
                self.zr6 = np.append(self.zr6, self.zc[a])
            elif self.ca[a] == 6:
                self.xr7 = np.append(self.xr7, self.xc[a])
                self.yr7 = np.append(self.yr7, self.yc[a])
                self.zr7 = np.append(self.zr7, self.zc[a])
            else:
                self.xr8 = np.append(self.xr8, self.xc[a])
                self.yr8 = np.append(self.yr8, self.yc[a])
                self.zr8 = np.append(self.zr8, self.zc[a])

        # Generate some n x 3 matrix by organizing the x, y, and z coordinates
        self.xyz1 = np.zeros((np.size(self.xr1), 3))
        self.xyz1[:, 0] = np.reshape(self.xr1, -1)
        self.xyz1[:, 1] = np.reshape(self.yr1, -1)
        self.xyz1[:, 2] = np.reshape(self.zr1, -1)

        self.xyz2 = np.zeros((np.size(self.xr2), 3))
        self.xyz2[:, 0] = np.reshape(self.xr2, -1)
        self.xyz2[:, 1] = np.reshape(self.yr2, -1)
        self.xyz2[:, 2] = np.reshape(self.zr2, -1)

        self.xyz3 = np.zeros((np.size(self.xr3), 3))
        self.xyz3[:, 0] = np.reshape(self.xr3, -1)
        self.xyz3[:, 1] = np.reshape(self.yr3, -1)
        self.xyz3[:, 2] = np.reshape(self.zr3, -1)

        self.xyz4 = np.zeros((np.size(self.xr4), 3))
        self.xyz4[:, 0] = np.reshape(self.xr4, -1)
        self.xyz4[:, 1] = np.reshape(self.yr4, -1)
        self.xyz4[:, 2] = np.reshape(self.zr4, -1)

        self.xyz5 = np.zeros((np.size(self.xr5), 3))
        self.xyz5[:, 0] = np.reshape(self.xr5, -1)
        self.xyz5[:, 1] = np.reshape(self.yr5, -1)
        self.xyz5[:, 2] = np.reshape(self.zr5, -1)

        self.xyz6 = np.zeros((np.size(self.xr6), 3))
        self.xyz6[:, 0] = np.reshape(self.xr6, -1)
        self.xyz6[:, 1] = np.reshape(self.yr6, -1)
        self.xyz6[:, 2] = np.reshape(self.zr6, -1)

        self.xyz7 = np.zeros((np.size(self.xr7), 3))
        self.xyz7[:, 0] = np.reshape(self.xr7, -1)
        self.xyz7[:, 1] = np.reshape(self.yr7, -1)
        self.xyz7[:, 2] = np.reshape(self.zr7, -1)

        self.xyz8 = np.zeros((np.size(self.xr8), 3))
        self.xyz8[:, 0] = np.reshape(self.xr8, -1)
        self.xyz8[:, 1] = np.reshape(self.yr8, -1)
        self.xyz8[:, 2] = np.reshape(self.zr8, -1)

        # Create the pcd for each scanned angle
        self.pcd = o3d.geometry.PointCloud();
        self.pcd.points = o3d.utility.Vector3dVector(self.xyz1)
        self.pcd2 = o3d.geometry.PointCloud();
        self.pcd2.points = o3d.utility.Vector3dVector(self.xyz2)
        self.pcd3 = o3d.geometry.PointCloud();
        self.pcd3.points = o3d.utility.Vector3dVector(self.xyz3)
        self.pcd4 = o3d.geometry.PointCloud();
        self.pcd4.points = o3d.utility.Vector3dVector(self.xyz4)
        self.pcd5 = o3d.geometry.PointCloud();
        self.pcd5.points = o3d.utility.Vector3dVector(self.xyz5)
        self.pcd6 = o3d.geometry.PointCloud();
        self.pcd6.points = o3d.utility.Vector3dVector(self.xyz6)
        self.pcd7 = o3d.geometry.PointCloud();
        self.pcd7.points = o3d.utility.Vector3dVector(self.xyz7)
        self.pcd8 = o3d.geometry.PointCloud();
        self.pcd8.points = o3d.utility.Vector3dVector(self.xyz8)

        # Calculate the necessary rotation for each angle of scan
        self.pcd.rotate(self.pcd.get_rotation_matrix_from_xyz((0, 0, 0)), center=(0, 0, 0))
        self.pcd2.rotate(self.pcd2.get_rotation_matrix_from_xyz((0, np.pi / 4, 0)), center=(0, 0, 0))
        self.pcd3.rotate(self.pcd3.get_rotation_matrix_from_xyz((0, np.pi / 2, 0)), center=(0, 0, 0))
        self.pcd4.rotate(self.pcd4.get_rotation_matrix_from_xyz((0, 3 * np.pi / 4, 0)), center=(0, 0, 0))
        self.pcd5.rotate(self.pcd5.get_rotation_matrix_from_xyz((0, np.pi, 0)), center=(0, 0, 0))
        self.pcd6.rotate(self.pcd6.get_rotation_matrix_from_xyz((0, 5 * np.pi / 4, 0)), center=(0, 0, 0))
        self.pcd7.rotate(self.pcd7.get_rotation_matrix_from_xyz((0, 3 * np.pi / 2, 0)), center=(0, 0, 0))
        self.pcd8.rotate(self.pcd8.get_rotation_matrix_from_xyz((0, 7 * np.pi / 4, 0)), center=(0, 0, 0))

        # Initialized a pcd for integrating all pcds into one
        self.pcd_combined = o3d.geometry.PointCloud()

        for i in range(9):
            if i == 0:
                self.pcd_combined += self.pcd
            elif i == 1:
                self.pcd_combined += self.pcd2
            elif i == 2:
                self.pcd_combined += self.pcd3
            elif i == 3:
                self.pcd_combined += self.pcd4
            elif i == 4:
                self.pcd_combined += self.pcd5
            elif i == 5:
                self.pcd_combined += self.pcd6
            elif i == 6:
                self.pcd_combined += self.pcd7
            else:
                self.pcd_combined += self.pcd8

        # This works around repeat points by creating a small radius voxel around each point and combining them into a singular point
        self.pcd_final = self.pcd.voxel_down_sample(voxel_size=0.05)  # Adjust Voxel size as needed

        # Convert Open3D.o3d.geometry.PointCloud to numpy array.
        o3d.visualization.draw([self.pcd_combined])
        self.xyz_converted = np.asarray(self.pcd_combined.points)
