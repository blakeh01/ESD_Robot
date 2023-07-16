import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib
matplotlib.use("TkAgg")

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
startOrientation = p.getQuaternionFromEuler([0, 0, -np.pi/2])    # point robot towards positive X axis
robotId = p.loadURDF("pantex/urdf/rx200pantex.urdf", startPos, startOrientation, globalScaling=1)
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)

print(p.getNumJoints(robotId))
for i in range(p.getNumJoints(robotId)):
    print(p.getJointInfo(robotId, i))

# Create a dynamixel robot instance
#rb_interface = RobotHandler([0, 0, 0])

# Create a test probing object
objStartPos = [0, 0.05, 0.25]
objStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
#objId = p.loadURDF("cube.urdf", objStartPos, objStartOrientation, globalScaling=0.5)
#p.resetBasePositionAndOrientation(objId, objStartPos, objStartOrientation)


# debug params:
speedParamID = p.addUserDebugParameter("action speed", 1, 10, 1)
#ex_selectionID = p.addUserDebugParameter("action selection", 1, 1, 1)

t=0.
ref_t=time.time()
dt=0.
while 1:
    t += dt
    print(f"Time elapsed: {t} s | dt: {dt} s")
    p.stepSimulation()
    time.sleep(1./SIM_FPS)

    for i in range(1):
        pos = [np.sin(t), 0.2, 0.3]
        orn = p.getQuaternionFromEuler([0, 0, 0])

        jointPoses = p.calculateInverseKinematics(robotId, 5, pos, orn)

    # index 0: linear actuator, index 2: waist, index 3: shoulder
    # index 4: elbow, index 5: wrist
    # todo figure why this is, and make it more intuitive
    #rb_interface.setJointPoses(jointPoses)
    p.setJointMotorControl2(robotId, 0,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=jointPoses[0],
                            targetVelocity=0,
                            force=500,
                            positionGain=0.1,
                            velocityGain=1)

    p.setJointMotorControl2(robotId, 2,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=jointPoses[1],
                            targetVelocity=0,
                            force=500,
                            positionGain=0.1,
                            velocityGain=1)

    p.setJointMotorControl2(robotId, 3,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=jointPoses[2],
                            targetVelocity=0,
                            force=500,
                            positionGain=0.03,
                            velocityGain=1)

    p.setJointMotorControl2(robotId, 4,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=jointPoses[3],
                            targetVelocity=0,
                            force=500,
                            positionGain=0.03,
                            velocityGain=1)

    p.setJointMotorControl2(robotId, 5,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=jointPoses[4],
                            targetVelocity=0,
                            force=500,
                            positionGain=0.03,
                            velocityGain=1)

    if t > 6000:
        break

    dt = time.time()-ref_t
    ref_t = time.time()

p.disconnect()

