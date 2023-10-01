import pybullet as p
import pybullet_planning as pp
import numpy as np

class ProbePositionSetter():

    def __init__(self, sim, goal_pos, timeout=0):
        self.sim = sim
        self.goal_pos = goal_pos
        self.complete = False
        self.point = None

    def onUpdate(self):
        if self.complete:
            return

        if not self.point: self.point = pp.draw_point(self.goal_pos)

        joint_poses = p.calculateInverseKinematics(self.sim.sim_robot, 6, self.goal_pos,
                                                   p.getQuaternionFromEuler([0, 0, 0]))#-np.pi / 2]))
        self.enableMotor(joint_poses, [1, 2, 3, 4, 5])

        self.complete = np.linalg.norm(np.array(self.goal_pos) - np.array(pp.get_link_pose(self.sim.sim_robot, 6)[0])) <= 0.01

    def enableMotor(self, joint_pos, joints):
        # awful code but no better solution atm

        # RAIL
        p.setJointMotorControl2(self.sim.sim_robot, joints[0],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[0],
                                force=1000,
                                positionGain=0.5,
                                velocityGain=1,
                                maxVelocity=0.125)

        # WAIST
        p.setJointMotorControl2(self.sim.sim_robot, joints[1],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[1],
                                force=1000,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=0)

        # SHOULDER
        p.setJointMotorControl2(self.sim.sim_robot, joints[2],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[2])

        # ELBOW
        p.setJointMotorControl2(self.sim.sim_robot, joints[3],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[3])

        # WRIST/PROBE
        p.setJointMotorControl2(self.sim.sim_robot, joints[4],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[4])

class PlatformPositionSetter():

    def __init__(self, sim, inc_rot, timeout=0):
        self.sim = sim
        self.goal_rot = inc_rot + pp.get_joint_position(self.sim.sim_platform, 1)
        self.complete = False

    def onUpdate(self):
        if self.complete:
            return

        if(abs(pp.get_joint_position(self.sim.sim_platform, 1) - self.goal_rot) <= 0.00005):
            print("Done!")
            self.complete = True

        p.setJointMotorControl2(self.sim.sim_platform, 1,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.goal_rot,
                                force=1000,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=0.5)
