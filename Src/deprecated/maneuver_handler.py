from robohelper import *


# A maneuver is defined as a single DXL moving to a specified coordinate. [absolute coordinate]
# This class stores the current maneuver list, and will execute this list whenever called.
# Executing a maneuver list will move the arm to a specified location, based on the maneuvers.
# It will not interpolate.
# It also stores the previous maneuver for proper pauses between maneuvers.
# Will properly move dynamixels ID 2 and ID 3 (main arm joint) together.


class ManeuverHandler:

    def __init__(self, packet_handler, port_handler, group_sync_write, maneuver_list=None, previous_maneuver=None):
        if maneuver_list is None:
            maneuver_list = []
        self.packet_handler = packet_handler
        self.port_handler = port_handler
        self.group_sync_write = group_sync_write
        self.maneuver_list = maneuver_list
        self.previous_maneuver = previous_maneuver

    def executeManeuverList(self):
        for maneuver in self.maneuver_list:
            self.executeManeuver(maneuver)

        self.clearManeuvers()

    # NOTE: if calling DXL_ID_02 or DXL_ID_03, THEY WILL EXECUTE SIMULTANEOUSLY.. ONLY CALL FOR ONE. TODO BETTER SOLUTION
    def executeManeuver(self, maneuver):
        # Set velocity to the passed in maneuver
        setVelocityProfile(self.packet_handler, self.port_handler, maneuver.dxl_id, maneuver.dt)
        print(f"[ID:{maneuver.dxl_id}] Beginning maneuver to {maneuver.end_pos}!")

        # Check to make sure this manuever will not overlap with a previous maneuver, if it does, wait out the previous.
        if self.previous_maneuver is not None and self.previous_maneuver.dxl_id == maneuver.dxl_id:
            time.sleep(self.previous_maneuver.dt / 1000)

        # If we are moving the main arm, both motors must be used to avoid overloading the servos.
        if maneuver.dxl_id is DXL_ID_02 or maneuver.dxl_id is DXL_ID_03:
            setVelocityProfile(self.packet_handler, self.port_handler, DXL_ID_02, maneuver.dt)
            setVelocityProfile(self.packet_handler, self.port_handler, DXL_ID_03, maneuver.dt)
            syncMoveToSamePosition(self.packet_handler, self.group_sync_write, DXL_ID_02, DXL_ID_03, maneuver.end_pos)
        else:
            writeGoalPosition(self.packet_handler, self.port_handler, maneuver.dxl_id, maneuver.end_pos, maneuver.dt)

        self.previous_maneuver = maneuver
        print(f"[ID:{maneuver.dxl_id}] Maneuver sent to DXL!")
        setVelocityProfile(self.packet_handler, self.port_handler, maneuver.dxl_id, VELOCITY_LIMIT_DEF)     # Reset velocity


    def addManeuver(self, maneuver):
        self.maneuver_list.append(maneuver)

    def removeManeuver(self, maneuver):
        self.maneuver_list.remove(maneuver)

    def getManeuvers(self):
        return self.maneuver_list

    def clearManeuvers(self):
        self.maneuver_list = []
        self.previous_maneuver = None


# a 'Maneuver' is a storage object containing a dynamixel ID (dynamixel to move), the goal end position, and the time it
# should take to reach that position.
class Maneuver:

    def __init__(self, dxl_id, end_pos, dt):
        self.dxl_id = dxl_id
        self.end_pos = end_pos
        self.dt = dt
