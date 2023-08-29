import roboticstoolbox as rtb
import spatialgeometry as geo
import swift
from src.deprecated.maneuver_handler import *
from src.deprecated.models.rx200_modified import rx200Modified

# DXL Instance Variables
# Create a new port handler instance under the device.
portHandler = PortHandler(DEVICENAME)

# Create a new packet handler instance with defined protocol (2.0)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Create a new maneuver handler instance
maneuverHandler = ManeuverHandler(packetHandler, portHandler, groupSyncWrite)

# Open port, terminate if no connection is established.
if portHandler.openPort():
    print(f"Successfully opened port on {DEVICENAME}!")
else:
    print(f"Failed to open port on {DEVICENAME}.")
    print("Press any key to terminate...")
    getch()
    quit()

# Set the baud rate, terminate under failure.
if portHandler.setBaudRate(BAUDRATE):
    print(f"Successfully set baud rate to {BAUDRATE} bps!")
else:
    print(f"Failed to set baud rate.")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable the Torque drive on given devices.
setTorque(packetHandler, portHandler, DXL_ID_01, TORQUE_ENABLE)
setTorque(packetHandler, portHandler, DXL_ID_02, TORQUE_ENABLE)
setTorque(packetHandler, portHandler, DXL_ID_03, TORQUE_ENABLE)
setTorque(packetHandler, portHandler, DXL_ID_04, TORQUE_ENABLE)
setTorque(packetHandler, portHandler, DXL_ID_05, TORQUE_ENABLE)

setVelocityProfile(packetHandler, portHandler, DXL_ID_02, 500)
setVelocityProfile(packetHandler, portHandler, DXL_ID_03, 500)
setVelocityProfile(packetHandler, portHandler, DXL_ID_04, 500)
setVelocityProfile(packetHandler, portHandler, DXL_ID_05, 500)

print("Press any key to go home! (or press ESC to quit!)")
if getch() == chr(0x1b):
    exit()

asynchronousGoHome(packetHandler, portHandler)

# Initialize RBT
robot = rx200Modified()
test_shape = geo.Cylinder(radius=.1, length=.25)
test_shape.T = sm.SE3(0, 0, .25 / 2)  # setting the 'pose' of the object. in essence, this is just the position
print(robot)
print(test_shape)

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
env.add(test_shape)

# Create 2 test trajectories
to_test_traj = rtb.jtraj(robot.qz, robot.qr, 5)
from_test_traj = rtb.jtraj(robot.qr, robot.qz, 5)

print(to_test_traj)

while 1:

    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    print("For a computer simulated maneuver, press [w]. For simulation mocking mode, press [m]. "
          "(or press ESC to quit!)")
    given = getch()

    if given == 'w':
        print("Beginning motor 4 test... press CTRL+C to exit.")
        time.sleep(3)
        while 1:
            motor_4_up = rtb.jtraj(robot.qz, np.array([0, np.pi / 4, 0, np.pi / 4]), 20)
            for qk in motor_4_up.q:  # for each joint configuration on trajectory
                robot.q = qk  # update the robot state
                matchSimulatedPosition(packetHandler, portHandler, groupSyncWrite, qk, 100)
                env.step()  # update visualization

            motor_4_down = rtb.jtraj(robot.q, robot.qz, 20)
            for qk in motor_4_down.q:
                robot.q = qk
                matchSimulatedPosition(packetHandler, portHandler, groupSyncWrite, qk, 100)
                env.step()

    elif given == 'm':
        print("Mocking robot in 3s... Disabling torque. CTRL+C to exit.")
        time.sleep(3)

        setTorque(packetHandler, portHandler, DXL_ID_01, TORQUE_DISABLE)
        setTorque(packetHandler, portHandler, DXL_ID_02, TORQUE_DISABLE)
        setTorque(packetHandler, portHandler, DXL_ID_03, TORQUE_DISABLE)
        setTorque(packetHandler, portHandler, DXL_ID_04, TORQUE_DISABLE)
        setTorque(packetHandler, portHandler, DXL_ID_05, TORQUE_DISABLE)

        while 1:
            waist_pos = readCurrentPosition(packetHandler, portHandler, DXL_ID_01)
            shoulder_pos = readCurrentPosition(packetHandler, portHandler, DXL_ID_02)
            elbow_pos = readCurrentPosition(packetHandler, portHandler, DXL_ID_04)
            wrist_pos = readCurrentPosition(packetHandler, portHandler, DXL_ID_05)

            waist_rot = positionUnitToRadians(waist_pos) - np.pi
            shoulder_rot = positionUnitToRadians(shoulder_pos - DXL_HOME[DXL_ID_02])
            elbow_rot = positionUnitToRadians(elbow_pos - DXL_HOME[DXL_ID_04])
            wrist_rot = positionUnitToRadians(-wrist_pos + DXL_HOME[DXL_ID_05])

            print(f"Mocking joint rotations: \n{waist_pos} -> {waist_rot} rad"
                  f"\n{shoulder_pos} -> {shoulder_rot} rad"
                  f"\n{elbow_pos} -> {elbow_rot} rad")

            pose = np.array([waist_rot, shoulder_rot, -elbow_rot, wrist_rot])
            robot.q = pose
            env.step(MODE_MOCK_STEP / 1000)

    elif given == chr(0x1b):
        break

# Disable Dynamixel Torque
setTorque(packetHandler, portHandler, DXL_ID_01, TORQUE_DISABLE)
setTorque(packetHandler, portHandler, DXL_ID_02, TORQUE_DISABLE)
setTorque(packetHandler, portHandler, DXL_ID_03, TORQUE_DISABLE)
setTorque(packetHandler, portHandler, DXL_ID_04, TORQUE_DISABLE)
setTorque(packetHandler, portHandler, DXL_ID_05, TORQUE_DISABLE)

# Close port
portHandler.closePort()

env.hold()
