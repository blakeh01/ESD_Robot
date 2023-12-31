'''

This file will read in the points from the pathing loop and figure out and delete any similar points

'''

import numpy as np
import open3d as o3d


#  from src.loop import ScannerPoints (eventually call from another file) ***Needs to bewritten to Z Coords and offset added***


def ScannedObjectCreation():
    # Rotation Angle
    degree = 45

    # Testing points (Comment these out or delete)
    xc = np.array([0, 5, 10, 0, 5, 10, 0, 5, 10, 10])
    yc = np.array([0, 5, 10, 10, 10, 30, 30, 30, 40, 40])
    zc = np.array([0, 25, 25, 25, 25, 25, 25, 25, 25, 25])
    ca = np.array([0, 1, 2, 3, 4, 5, 6, 7, 7, 0])

    # Initialize the arrays
    xr1 = np.array([]);
    yr1 = np.array([]);
    zr1 = np.array([])
    xr2 = np.array([]);
    yr2 = np.array([]);
    zr2 = np.array([])
    xr3 = np.array([]);
    yr3 = np.array([]);
    zr3 = np.array([])
    xr4 = np.array([]);
    yr4 = np.array([]);
    zr4 = np.array([])
    xr5 = np.array([]);
    yr5 = np.array([]);
    zr5 = np.array([])
    xr6 = np.array([]);
    yr6 = np.array([]);
    zr6 = np.array([])
    xr7 = np.array([]);
    yr7 = np.array([]);
    zr7 = np.array([])
    xr8 = np.array([]);
    yr8 = np.array([]);
    zr8 = np.array([])

    # Categorize the different scanned points into their correct scanned angle
    for a in range(0, (np.size(ca))):
        if ca[a] == 0:
            xr1 = np.append(xr1, xc[a]);
            yr1 = np.append(yr1, yc[a]);
            zr1 = np.append(zr1, zc[a])
        elif ca[a] == 1:
            xr2 = np.append(xr2, xc[a]);
            yr2 = np.append(yr2, yc[a]);
            zr2 = np.append(zr2, zc[a])
        elif ca[a] == 2:
            xr3 = np.append(xr3, xc[a]);
            yr3 = np.append(yr3, yc[a]);
            zr3 = np.append(zr3, zc[a])
        elif ca[a] == 3:
            xr4 = np.append(xr4, xc[a]);
            yr4 = np.append(yr4, yc[a]);
            zr4 = np.append(zr4, zc[a])
        elif ca[a] == 4:
            xr5 = np.append(xr5, xc[a]);
            yr5 = np.append(yr5, yc[a]);
            zr5 = np.append(zr5, zc[a])
        elif ca[a] == 5:
            xr6 = np.append(xr6, xc[a]);
            yr6 = np.append(yr6, yc[a]);
            zr6 = np.append(zr6, zc[a])
        elif ca[a] == 6:
            xr7 = np.append(xr7, xc[a]);
            yr7 = np.append(yr7, yc[a]);
            zr7 = np.append(zr7, zc[a])
        else:
            xr8 = np.append(xr8, xc[a]);
            yr8 = np.append(yr8, yc[a]);
            zr8 = np.append(zr8, zc[a])

    # Generate some n x 3 matrix by organizing the x, y, and z coordinates
    xyz1 = np.zeros((np.size(xr1), 3))
    xyz1[:, 0] = np.reshape(xr1, -1);
    xyz1[:, 1] = np.reshape(yr1, -1);
    xyz1[:, 2] = np.reshape(zr1, -1)
    xyz2 = np.zeros((np.size(xr2), 3))
    xyz2[:, 0] = np.reshape(xr2, -1);
    xyz2[:, 1] = np.reshape(yr2, -1);
    xyz2[:, 2] = np.reshape(zr2, -1)
    xyz3 = np.zeros((np.size(xr3), 3))
    xyz3[:, 0] = np.reshape(xr3, -1);
    xyz3[:, 1] = np.reshape(yr3, -1);
    xyz3[:, 2] = np.reshape(zr3, -1)
    xyz4 = np.zeros((np.size(xr4), 3))
    xyz4[:, 0] = np.reshape(xr4, -1);
    xyz4[:, 1] = np.reshape(yr4, -1);
    xyz4[:, 2] = np.reshape(zr4, -1)
    xyz5 = np.zeros((np.size(xr5), 3))
    xyz5[:, 0] = np.reshape(xr5, -1);
    xyz5[:, 1] = np.reshape(yr5, -1);
    xyz5[:, 2] = np.reshape(zr5, -1)
    xyz6 = np.zeros((np.size(xr6), 3))
    xyz6[:, 0] = np.reshape(xr6, -1);
    xyz6[:, 1] = np.reshape(yr6, -1);
    xyz6[:, 2] = np.reshape(zr6, -1)
    xyz7 = np.zeros((np.size(xr7), 3))
    xyz7[:, 0] = np.reshape(xr7, -1);
    xyz7[:, 1] = np.reshape(yr7, -1);
    xyz7[:, 2] = np.reshape(zr7, -1)
    xyz8 = np.zeros((np.size(xr8), 3))
    xyz8[:, 0] = np.reshape(xr8, -1);
    xyz8[:, 1] = np.reshape(yr8, -1);
    xyz8[:, 2] = np.reshape(zr8, -1)

    # Create the pcd for each scanned angle
    pcd = o3d.geometry.PointCloud();
    pcd.points = o3d.utility.Vector3dVector(xyz1)
    pcd2 = o3d.geometry.PointCloud();
    pcd2.points = o3d.utility.Vector3dVector(xyz2)
    pcd3 = o3d.geometry.PointCloud();
    pcd3.points = o3d.utility.Vector3dVector(xyz3)
    pcd4 = o3d.geometry.PointCloud();
    pcd4.points = o3d.utility.Vector3dVector(xyz4)
    pcd5 = o3d.geometry.PointCloud();
    pcd5.points = o3d.utility.Vector3dVector(xyz5)
    pcd6 = o3d.geometry.PointCloud();
    pcd6.points = o3d.utility.Vector3dVector(xyz6)
    pcd7 = o3d.geometry.PointCloud();
    pcd7.points = o3d.utility.Vector3dVector(xyz7)
    pcd8 = o3d.geometry.PointCloud();
    pcd8.points = o3d.utility.Vector3dVector(xyz8)

    # Calculate the necessary rotation for each angle of scan
    pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, 0, 0)), center=(0, 0, 0))
    pcd2.rotate(pcd2.get_rotation_matrix_from_xyz((0, np.pi / 4, 0)), center=(0, 0, 0))
    pcd3.rotate(pcd3.get_rotation_matrix_from_xyz((0, np.pi / 2, 0)), center=(0, 0, 0))
    pcd4.rotate(pcd4.get_rotation_matrix_from_xyz((0, 3 * np.pi / 4, 0)), center=(0, 0, 0))
    pcd5.rotate(pcd5.get_rotation_matrix_from_xyz((0, np.pi, 0)), center=(0, 0, 0))
    pcd6.rotate(pcd6.get_rotation_matrix_from_xyz((0, 5 * np.pi / 4, 0)), center=(0, 0, 0))
    pcd7.rotate(pcd7.get_rotation_matrix_from_xyz((0, 3 * np.pi / 2, 0)), center=(0, 0, 0))
    pcd8.rotate(pcd8.get_rotation_matrix_from_xyz((0, 7 * np.pi / 4, 0)), center=(0, 0, 0))

    # Initialized a pcd for integrating all of the pcds into one
    pcd_combined = o3d.geometry.PointCloud()

    for i in range(9):
        if i == 0:
            pcd_combined += pcd
        elif i == 1:
            pcd_combined += pcd2
        elif i == 2:
            pcd_combined += pcd3
        elif i == 3:
            pcd_combined += pcd4
        elif i == 4:
            pcd_combined += pcd5
        elif i == 5:
            pcd_combined += pcd6
        elif i == 6:
            pcd_combined += pcd7
        else:
            pcd_combined += pcd8

    # This works around repeat point by creating a small radius voxel around each point and combining them into a singular point
    pcd_final = pcd.voxel_down_sample(voxel_size=0.05)  # Adjust Voxel size as needed

    # Convert Open3D.o3d.geometry.PointCloud to numpy array.
    o3d.visualization.draw([pcd_combined])
    xyz_converted = np.asarray(pcd_combined.points)

    return (pcd_final)
