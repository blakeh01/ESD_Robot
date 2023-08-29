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

setVelocityProfile(packetHandler, portHandler, DXL_ID_02, 1500)
setVelocityProfile(packetHandler, portHandler, DXL_ID_03, 1500)
setVelocityProfile(packetHandler, portHandler, DXL_ID_04, 1500)
setVelocityProfile(packetHandler, portHandler, DXL_ID_05, 1500)

print("Press any key to go home! (or press ESC to quit!)")
if getch() == chr(0x1b):
    exit()

asynchronousGoHome(packetHandler, portHandler)

# Initialize RBT
robot = rx200Modified()
pointer = geo.Sphere(radius=.05)
pointer.T = sm.SE3(0, 0, .25)  # setting the 'pose' of the object. in essence, this is just the position
print(robot)
print(pointer)

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
env.add(pointer)

print("Use WA(X)SD(Y)QZ(Z) to move target point. Press F to submit.")

MOVE_STEP_AMT = 0.05

currentPos = np.zeros(3)  # Position matrix
currentPose = pointer.T  # World matrix todo clean
print(f"Start position: {currentPos}")

while 1:
    given = getch()

    currentPos = [pointer.T[0][3], pointer.T[1][3], pointer.T[2][3]]

    if given == 'w':
        currentPos[0] = currentPos[0] + MOVE_STEP_AMT
        currentPose = currentPose + sm.SE3(MOVE_STEP_AMT, 0, 0)
    elif given == 's':
        currentPos[0] = currentPos[0] - MOVE_STEP_AMT
        currentPose = currentPose + sm.SE3(-MOVE_STEP_AMT, 0, 0)
    elif given == 'a':
        currentPos[1] = currentPos[1] + MOVE_STEP_AMT
        currentPose = currentPose + sm.SE3(0, MOVE_STEP_AMT, 0)
    elif given == 'd':
        currentPos[1] = currentPos[1] - MOVE_STEP_AMT
        currentPose = currentPose + sm.SE3(0, -MOVE_STEP_AMT, 0)
    elif given == 'q':
        currentPos[2] = currentPos[2] + MOVE_STEP_AMT
        currentPose = currentPose + sm.SE3(0, 0, MOVE_STEP_AMT)
    elif given == 'z':
        currentPos[2] = currentPos[2] - MOVE_STEP_AMT
        currentPose = currentPose + sm.SE3(0, 0, -MOVE_STEP_AMT)
    elif given == 'f':
        break

    print(f"Current position: {currentPos}")
    pointer.T = currentPose
    env.step()

print(f"Solving INVERSE KINEMATICS for point {currentPos}.")
start = time.process_time_ns()

T = sm.SE3(currentPos[0], currentPos[1], currentPos[2]) * sm.SE3.Rx(0, 'deg')  # create transformation matrix
print(type(T))

sol = robot.ikine_LM(T)  #
inv_traj = rtb.jtraj(robot.q, sol.q, 25)

print(f"Transformation matrix:\n{T}")
print(f"IKINE solution tuple: \n{sol}")
print(inv_traj)

end = time.process_time_ns()
print(f"Took {(end - start) / 1000000} ms to calculate the inverse trajectory.")

print("Press any key to begin trajectory. (or ESC to quit)")
if getch() == chr(0x1b):
    exit()

print("Moving robot to desired end point.")
for qk in inv_traj.q:  # for each joint configuration on trajectory
    robot.q = qk  # update the robot state
    # matchSimulatedPosition(packetHandler, portHandler, groupSyncWrite, qk, 50)
    env.step()  # update visualization

print("Arrived! Press any key to quit...")
getch()

# Disable Dynamixel Torque
setTorque(packetHandler, portHandler, DXL_ID_01, TORQUE_DISABLE)
setTorque(packetHandler, portHandler, DXL_ID_02, TORQUE_DISABLE)
setTorque(packetHandler, portHandler, DXL_ID_03, TORQUE_DISABLE)
setTorque(packetHandler, portHandler, DXL_ID_04, TORQUE_DISABLE)
setTorque(packetHandler, portHandler, DXL_ID_05, TORQUE_DISABLE)

# Close port
portHandler.closePort()
