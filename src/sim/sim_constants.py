import numpy as np

# Simulation constants

# Simulation info:
PYBULLET_SHOW_GUI = False
DRAW_TIP_AXES = True
SIM_SCALE = 2
UPDATE_RATE = 90  # Simulation speed / robot polling in frames per second.

SIM_ROBOT_OFFSET = np.dot(SIM_SCALE, [-0.40, 0, 0])
SIM_ROBOT_ORN = [0, 0, 0]  # Euler angles

SIM_PLATFORM_OFFSET = np.dot(SIM_SCALE, [0, 0, 0])
SIM_PLATFORM_ORN = [0, 0, 0]

SIM_CHAMBER_OFFSET = np.dot(SIM_SCALE, [0, 0, 0])
SIM_CHAMBER_ORN = [0, 0, np.pi / 2]

SIM_SCANNER_OFFSET = np.dot(SIM_SCALE, [-0.2, -0.27, 0])
SIM_SCANNER_ORN = [0, 0, 0]

# Normal Algorithm Constants and Constraints:
SCAN_CENTER = (0, 0)  # Where is the CoM of the object (x,y)?
RESOLUTION = 180  # How many ray casts. 360 = 1 every degree of the object
Z_LIMITS = (0.32, 0.485)  # Where should the scan begin and end.
Z_DENSITY = 20  # how many rays should be present on the Z (up/down) axis per degree of resolution
XZ_OFFSET = 0.2032  # how far away should the ray start
PROBE_DIST = 0.005 * SIM_SCALE  # how far should the point be from the object in mm
NORM_LENGTH = 0.05  # *for debug only* how long should the normal vectors be?
