# Default com ports:
### ROBOT ###
RBT_PORT = "COM6"
RBT_BAUD = 4000000

### STEPPER MOTORS ###
STEPPER_PORT = "COM4"
STEPPER_BAUD = 115200

### FEATHER 0 ###
FEATHER_PORT = "COM10"
FEATHER_BAUD = 9600

### LDS ###
LDS_PORT = "COM7"
LDS_BAUD = 115200

class PortConfiguration:

    def __init__(self):
        self.rbt_port = RBT_PORT
        self.rbt_baud = RBT_BAUD

        self.stepper_port = STEPPER_PORT
        self.stepper_baud = STEPPER_BAUD

        self.feather_port = FEATHER_PORT
        self.feather_baud = FEATHER_BAUD

        self.lds_port = LDS_PORT
        self.lds_baud = LDS_BAUD

    def update_rbt_settings(self, port, baud):
        self.rbt_port = port
        self.rbt_baud = baud

    def update_stepper_settings(self, port, baud):
        self.stepper_port = port
        self.stepper_baud = baud

    def update_feather_settings(self, port, baud):
        self.feather_port = port
        self.feather_baud = baud

    def update_lds_settings(self, port, baud):
        self.lds_port = port
        self.lds_baud = baud
