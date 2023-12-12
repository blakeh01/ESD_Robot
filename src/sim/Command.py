import numpy as np
import pybullet as p
import pybullet_planning as pp


class ProbePositionSetter:

    def __init__(self, sim, goal_pos, goal_orn, probe_v=2, a_tol=0.01, timeout=0):
        self.sim = sim
        self.goal_pos = goal_pos
        self.goal_orn = goal_orn
        self.complete = False
        self.point = None

        self.probe_v = probe_v
        self.a_tol = a_tol

    def onUpdate(self):
        if self.complete:
            return

        if not self.point: self.point = pp.draw_point(self.goal_pos)

        joint_poses = p.calculateInverseKinematics(self.sim.sim_robot, 6, self.goal_pos,
                                                   p.getQuaternionFromEuler(self.goal_orn))  # -np.pi / 2]))
        self.enableMotor(joint_poses, [1, 2, 3, 4, 5])

        self.complete = np.linalg.norm(np.array(self.goal_pos) - np.array(pp.get_link_pose(self.sim.sim_robot, 6)[
                                                                              0])) <= self.a_tol  # and np.linalg.norm(np.array(joint_poses[1:]) - np.array(self.sim.robot_handler.read_cur_conf())) <= 0.1

    def enableMotor(self, joint_pos, joints):
        # self.sim.robot_handler.set_goal_conf(joint_pos)

        # RAIL
        p.setJointMotorControl2(self.sim.sim_robot, joints[0],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[0],
                                force=1000,
                                positionGain=0.5,
                                velocityGain=1,
                                maxVelocity=0.1)

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
                                targetPosition=joint_pos[2],
                                maxVelocity=self.probe_v)

        # ELBOW
        p.setJointMotorControl2(self.sim.sim_robot, joints[3],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[3],
                                maxVelocity=self.probe_v)

        # WRIST/PROBE
        p.setJointMotorControl2(self.sim.sim_robot, joints[4],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[4],
                                maxVelocity=self.probe_v)


class PlatformPositionSetter():

    def __init__(self, sim, inc_rot, plat_v=0.5, timeout=0):
        self.sim = sim
        self.goal_rot = inc_rot + pp.get_joint_position(self.sim.sim_platform, 1)
        self.complete = False

        self.plat_v = plat_v

    def onUpdate(self):
        if self.complete:
            return

        if (abs(pp.get_joint_position(self.sim.sim_platform, 1) - self.goal_rot) <= 0.00005):
            self.complete = True

        # self.sim.robot_handler.stepper_board.write_a(np.rad2deg(self.goal_rot), feed=1600)
        p.setJointMotorControl2(self.sim.sim_platform, 1,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.goal_rot,
                                force=1000,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=self.plat_v)
