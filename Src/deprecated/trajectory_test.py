import pybullet as p
import pybullet_data
import matplotlib
from src.robot.robohelper import *

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from src.sim.sim_constants import SIM_FPS

# Instantiate and start a PyBullet physics client
physClient = p.connect(p.SHARED_MEMORY, options="--opengl3")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

# Load ground plane into environment.
planeObj = p.loadURDF("plane.urdf")

# Create the robot in PyBullet environment
# index 0 is base, 1 -> shoulder, 2 -> elbow, 3 -> wrist
startPos = [0, 0.5334, 0.0333375]
startOrientation = p.getQuaternionFromEuler([0, 0, -np.pi / 2])  # point robot towards positive X axis
robotId = p.loadURDF("pantex/urdf/rx200pantex.urdf", startPos, startOrientation, globalScaling=1)
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)

# Add rotating baseplate to environment
platePos = [0, 0, 0.0635]
plateOrientation = p.getQuaternionFromEuler([0, 0, 0])
baseId = p.loadURDF("pantex/urdf/rot_base.urdf", platePos, plateOrientation)
p.resetBasePositionAndOrientation(baseId, platePos, plateOrientation)

# create a bunch of test via points.
box_min = [0.25, -0.75, 0.25]
box_max = [-0.25, -0.75, 0.75]

num_x = 15
num_z = 15

interval_x = np.linspace(box_min[0], box_max[0], num_x)
interval_z = np.linspace(box_min[2], box_max[2], num_z)

# Bigger corner points
p.addUserDebugPoints([box_min], [[255, 0, 0]], pointSize=10)
p.addUserDebugPoints([box_max], [[255, 0, 0]], pointSize=10)

# Compile list of via points
points = []

for i in range(0, num_x):
    if i%2==0:
        for k in range(0, num_z):
            points.append([interval_x[i], box_max[1], interval_z[k]])
    else:
        for k in range(num_z-1, 0, -1):
            points.append([interval_x[i], box_max[1], interval_z[k]])


# index 0: linear actuator, index 2: waist, index 3: shoulder
# index 4: elbow, index 5: wrist
jointPoses = []
positionList = {"actuator": [], "waist": [], "shoulder": [], "elbow": [], "wrist": []}
orn = p.getQuaternionFromEuler([0, 0, 0])  # keep affector 'level'

for point in points:
    p.addUserDebugPoints([[point[0], point[1], point[2]]], [[0, 255, 0]], pointSize=3)
    jointPoses.append(p.calculateInverseKinematics(robotId, 5, [point[0], point[1], point[2]], orn))

for pose in jointPoses:
    positionList["actuator"].append(radiansToPositionUnit(pose[0]))
    positionList["waist"].append(radiansToPositionUnit(pose[1]))
    positionList["shoulder"].append(radiansToPositionUnit(pose[2]))
    positionList["elbow"].append(radiansToPositionUnit(pose[3]))
    positionList["wrist"].append(radiansToPositionUnit(pose[4]))

for pos in positionList:
    print(positionList[pos])

x = np.arange(0, len(jointPoses))
y = []

plt.figure(0)
y = positionList["actuator"]
plt.title("Actuator Position")
plt.plot(x, y)

plt.figure(1)
y = positionList["waist"]
plt.title("Waist Position")
plt.plot(x, y)

plt.figure(2)
y = positionList["shoulder"]
plt.title("Shoulder Position")
plt.plot(x, y)

plt.figure(3)
y = positionList["elbow"]
plt.title("Elbow Position")
plt.plot(x, y)

plt.figure(4)
y = positionList["wrist"]
plt.title("Wrist Position")
plt.plot(x, y)

plt.show()

while 1:
    p.stepSimulation()
    time.sleep(1./SIM_FPS)


def movementLoop(jointPoses, tolerance):
    p.setJointMotorControlArray(robotId, [0, 2, 3, 4, 5],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=jointPoses,
                                forces=[5000, 5000, 5000, 5000, 5000],
                                positionGains=[0.1, 0.1, 0.1, 0.1, 0.1],
                                velocityGains=np.ones(5))
    arrived = False
    while not arrived:
        numArrived = 0
        currentStates = getNeedyStates()

        for i in range(len(currentStates)):
            cur_pos = currentStates[i][0]
            goal_pos = jointPoses[i]
            if abs(cur_pos - goal_pos) < tolerance:
                numArrived += 1
                # print(f"Joint arrived in goal state! {numArrived}/{len(currentStates)}")

        if numArrived is len(currentStates):
            print("Arrived!")
            arrived = True

        updateVars()
        drawProbeRay()

# Returns the 'needy' joint states. That is, ones that are required in inverse kinematic calculations.
def getNeedyStates():
    return [p.getJointState(robotId, 0), p.getJointState(robotId, 2), p.getJointState(robotId, 3),
            p.getJointState(robotId, 4), p.getJointState(robotId, 5)]


def drawProbeRay():
    probe_affector_pos = p.getLinkState(robotId, 6)[0]
    p.addUserDebugLine(probe_affector_pos, [probe_affector_pos[0], -1, probe_affector_pos[2] - 0.02], lifeTime=0.05,
                       lineColorRGB=[1, 0, 0], lineWidth=2.5)

def updateVars():
    p.stepSimulation()
    time.sleep(1. / SIM_FPS)


while 1:
    updateVars()
    drawProbeRay()

    movementLoop(p.calculateInverseKinematics(robotId, 5, [0, objStartPos[1] + 0.8, objStartPos[2]], p.getQuaternionFromEuler([0, 0, 0])), 0.0001)

    for pose in jointPoses:
        movementLoop(pose, 0.0001)


p.disconnect()
