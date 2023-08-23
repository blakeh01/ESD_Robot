'''

    Using pybullets raycasting system to generate uniform point clouds.

    This will replace the old slicing system with Open3D voxelization, simply as its more reliable, configurable, and
    better on the CPU.

    REF: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA
'''
import pybullet as p
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
import math
import time
from dataclasses import dataclass

@dataclass
class AlignmentData:
    position: list
    normal:   list

def generate_slices(filePath, scale, obj_pos=[0,0,0], obj_eorn=[0, 0, 0], obj_center=[0, 0, 0],
                    object_height=0.01, raycast_distance=0.1, raycast_XY_density=180, raycast_Z_density=30, probe_dist_mm=1,
                    plot=False):
    ## CONNECT TO PYBULLET
    client = p.connect(p.GUI)

    ## CREATE COLLISION SHAPE MESH (mass = 0)
    cuid = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                  fileName=filePath,
                                  meshScale=scale,
                                  flags=p.GEOM_FORCE_CONCAVE_TRIMESH | p.GEOM_CONCAVE_INTERNAL_EDGE)
    object = p.createMultiBody(0, cuid)
    p.resetBasePositionAndOrientation(object, obj_pos, p.getQuaternionFromEuler(obj_eorn))

    ## CREATE RAY CASTING PARAMETERS (via. cylindrical cooridnates)
    ## REF: https://en.wikipedia.org/wiki/Cylindrical_coordinate_system
    assert raycast_XY_density * raycast_Z_density <= p.MAX_RAY_INTERSECTION_BATCH_SIZE

    rho         = raycast_distance
    phi_space   =  np.linspace(0, 2*np.pi, num=raycast_XY_density, endpoint=False)
    z_space     = np.linspace(0, object_height, num=raycast_Z_density)

    ## CREATE ARRAY OF START POSITIONS IN CARTESIAN COORDINATES
    raycast_start_arr = []
    raycast_end_arr = []

    for phi in phi_space:
        for z in z_space:
            raycast_start_arr.append((rho*np.cos(phi), rho*np.sin(phi), z))
            raycast_end_arr.append((obj_center[0], obj_center[1], z))

    p.addUserDebugPoints(raycast_start_arr, [[255, 0, 0]]*len(raycast_start_arr), 2)

    ## BATCH CAST RAYS, FILTER, CREATE ALIGNMENT POINT DATA.
    results = p.rayTestBatch(raycast_start_arr, raycast_end_arr, numThreads=0)

    alignment_points = []

    for r in results:
        if r[0] == object:
            hit_pos = r[3]
            hit_norm = r[4]

            offset_pos = np.add(hit_pos, np.dot(hit_norm, probe_dist_mm/1000))
            alignment_points.append(AlignmentData(offset_pos, hit_norm))

    slices = slice(alignment_points)
    if plot: plot_slices(slices)

    # DISCONNECT FROM BYPULLET
    p.disconnect()

    return slices


def slice(alignment_points, tolerance=0.001):
    sliced_dict = {}

    for data in alignment_points:
        # Get the z-axis position of the alignment data
        z_pos = data.position[2]

        # Check if the z-axis position falls within the tolerance range of an existing key in the dictionary
        grouped_key = None
        for key in sliced_dict.keys():
            if abs(z_pos - key) <= tolerance:
                grouped_key = key
                break

        # If the z-axis position falls within the tolerance range of an existing key, append the alignment data to the corresponding list
        if grouped_key is not None:
            sliced_dict[grouped_key].append(data)
        else:
            # If the z-axis position does not fall within the tolerance range of an existing key, add it as a new key and create a new list as its value
            sliced_dict[z_pos] = [data]

    return sliced_dict


def plot_slices(sliced_dict):
    # Create a figure and axes for the plot and slider
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25) # Adjust the bottom margin to make space for the slider
    slider_ax = plt.axes([0.2, 0.1, 0.6, 0.05]) # Define the location and size of the slider axes

    # Create a slider widget for the slice index
    slice_slider = Slider(slider_ax, 'Slice Index', 0, len(sliced_dict)-1, valinit=0, valstep=1, facecolor='#cc7000')

    # Define a function to update the plot when the slider value changes
    def update_plot(val):
        # Get the list of instances corresponding to the current slider value
        slice = sliced_dict[list(sliced_dict.keys())[int(val)]]

        # Extract the X and Y coordinates from the position attribute of each instance
        x_coords = [data.position[0] for data in slice]
        y_coords = [data.position[1] for data in slice]

        # Clear the current plot and plot the updated X and Y coordinates with color-coded labels
        ax.clear()
        scatter = ax.scatter(x_coords, y_coords)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title(f'XY Coords of Slice {int(val)}')

        plt.draw()

    # Connect the slider widget to the update_plot function
    slice_slider.on_changed(update_plot)
    update_plot(0)

    plt.show()