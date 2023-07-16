### DYNAMIXELS ###
# Addresses / Address length
ADDR_TORQUE_ENABLE          = 64                            # Control table addresses for XL-430
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4                             # Number of bytes
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4

ADDR_PROF_VELOCITY          = 112
VELOCITY_LIMIT_DEF          = 200                            # Default time per movement command.

# Protocol version
PROTOCOL_VERSION            = 2

# Default vars
BAUDRATE                    = 1000000
DEVICENAME                  = "COM3"                        # Check which port is being used on your controller

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

UNIT_POS_TO_DEG             = 0.087891                      # 1 unit pos = 0.087891 degrees.



### STEPPER MOTORS ###
STEPPER_PORT                  = "COM8"
STEPPER_BAUD                  = 115200
