# Addresses / Address length
ADDR_TORQUE_ENABLE = 64  # Control table addresses for XL-430
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4  # Number of bytes
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

ADDR_PRESENT_LOAD = 126
LEN_PRESENT_LOAD = 2

ADDR_PROF_ACCEL = 108
ADDR_PROF_VELOCITY = 112
ADDR_POS_P_GAIN = 84
ADDR_POS_I_GAIN = 82
ADDR_POS_D_GAIN = 80

# Protocol version
PROTOCOL_VERSION = 2

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

COMM_SUCCESS = 0  # Communication Success result value
COMM_TX_FAIL = -1001  # Communication Tx Failed

UNIT_POS_TO_DEG = 0.087891  # 1 unit pos = 0.087891 degrees.

DXL_IDS = [1, 2, 3, 4, 5]
