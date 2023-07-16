from src.sim.sim_constants import *
from roboticstoolbox.robot.ERobot import ERobot


class rx200Modified(ERobot):

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "interbotix_descriptions/urdf/rx_200_modified.udrf.xacro"
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Interbotix",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            base=ROBOT_BASE_OFFSET
        )

        self.qr = np.array([0, -np.pi/4, -np.pi/4, np.pi/4])
        self.qz = np.zeros(4)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
