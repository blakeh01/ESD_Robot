import time

import numpy as np
import pybullet as p
import src.sim.simhelper as simhelper


class Command:

    def __init__(self, controller, timeout=0):
        self.controller_instance = controller

        self.sim_instance = controller.simulation_instance
        self.rbt_instance = controller.robot_instance
        self.timeout = timeout

        self.complete = False
        self.args = []

    def executeCommand(self, *args):
        self.args = args
        self.complete = False

        if self.controller_instance.current_command == None:
            self.controller_instance.current_command = self
        else:
            print("[CMD] ERROR! Attempted to execute command while one is being processed!")
            return False

        return True

    def onUpdate(self):
        if self.complete:
            self.controller_instance.current_command = None
            print("[CMD] Command Complete!")
            return


class CmdTest(Command):
    def __init__(self, controller, timeout=0):
        super().__init__(controller, timeout)

        self.step = 0
        self.plan = None
        self.frame = 0
        self.pc_rot = 0
        self.line_id = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 0, 1], 5)
        self.refid = p.addUserDebugPoints([[0, 0, 0]], [[0, 0, 0]], 5)

        self.waypoints = [
            [0, 0.0925, 0.2 + 0.2],
            [0, 0.0925, 0.22 + 0.2],
            [0, 0.0925, 0.24 + 0.2],
            [0, 0.0925, 0.26 + 0.2],
            [0, 0.0925, 0.28 + 0.2],
            [0, 0.0925, 0.3 + 0.2],
            [0, 0.0925, 0.32 + 0.2],
            [0, 0.0925, 0.34 + 0.2],
            [0, 0.0925, 0.36 + 0.2],
            [0, 0.0925, 0.38 + 0.2]
        ]
        p.addUserDebugPoints(self.waypoints, [[1, 0, 0]] * len(self.waypoints), 2)

        self.is_here = True
        self.goal_conf = None
        self.do_spin = False
        self.is_spinning = False
        self.new = True
        self.spin_waiter = 0
        self.WAITSPIN = 10
        self.current_idx = 9

    def executeCommand(self, *args):
        if super().executeCommand(args):
            print("[CMD] Doing test!")

    def onUpdate(self):
        super().onUpdate()

        def spin():
            self.do_spin = True

        def go_to_next(index):
            for i in range(0, 5):
                conf = p.calculateInverseKinematics(self.sim_instance.sim_robot, 6, self.waypoints[index],
                                                    p.getQuaternionFromEuler([0, 0, -np.pi / 2]))
            self.is_here = False
            return conf

        if not self.is_here and np.isclose(self.sim_instance.get_sim_joint_states(), self.goal_conf, atol=0.005).all():
            self.is_here = True
            spin()

        if not self.is_here:
            p.setJointMotorControlArray(self.sim_instance.sim_robot, [0, 2, 3, 4, 5],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=self.goal_conf,
                                        forces=[1000, 1000, 1000, 1000, 1000],
                                        positionGains=[0.05, 0.05, 0.05, 0.05, 0.05],
                                        velocityGains=[0.5, 0.5, 0.5, 0.5, 0.5])

        if self.do_spin:
            self.rbt_instance.stepper_handler.spin_plat()
            print("Sending spin!")
            self.is_spinning = True
            self.do_spin = False

        if self.is_spinning:
            self.spin_waiter += 1

        if self.spin_waiter >= 90 * (self.WAITSPIN) and self.is_spinning:
            self.is_spinning = False
            self.current_idx -= 1
            self.goal_conf = go_to_next(self.current_idx)
            self.spin_waiter = 0

        if self.new:
            self.goal_conf = go_to_next(self.current_idx)
            self.new = False

        # self.complete = True

        # self.frame += 1

        #
        # if self.plan is None:
        #     pp.set_joint_positions(
        #         self.sim_instance.sim_robot,
        #         self.sim_instance.robot_joints,
        #         p.calculateInverseKinematics(self.sim_instance.sim_robot, 6, [0, 0, 1],
        #                                      p.getQuaternionFromEuler([0, 0, -np.pi / 2]))
        #     )
        #
        #     self.plan = pp.plan_joint_motion(
        #         self.sim_instance.sim_robot,
        #         self.sim_instance.robot_joints,
        #         p.calculateInverseKinematics(self.sim_instance.sim_robot, 6, [0, 0.2, 0.5],
        #                                      p.getQuaternionFromEuler([0, 0, -np.pi / 2])),
        #         resolutions=0.025
        #     )
        #
        #     pp.set_joint_positions(
        #         self.sim_instance.sim_robot,
        #         self.sim_instance.robot_joints,
        #         p.calculateInverseKinematics(self.sim_instance.sim_robot, 6, [0, 0, 1],
        #                                      p.getQuaternionFromEuler([0, 0, -np.pi / 2]))
        #     )
        #
        # path = pp.plan_cartesian_motion(self.sim_instance.sim_robot, [])

        # pp.simulate_controller(ctrl)

        # self.complete = True
        #
        # if self.frame % 5 == 0:
        #     pp.set_joint_positions(
        #         self.sim_instance.sim_robot,
        #         self.sim_instance.robot_joints,
        #         self.plan[self.step]
        #     )
        #     self.step += 1
        #
        # if self.step == len(self.plan):
        #     self.step = 0
        #     self.frame = 0
        #     self.plan = None
        #     self.complete = True

        # waypoints = [
        #     pp.Pose([0, 0, 0.8], [0, 0, -np.pi/2]),
        #     pp.Pose([0, 0, 0.9], [0, 0, -np.pi/2]),
        #     pp.Pose([0, 0, 0.8], [0, 0, -np.pi/2]),
        #     pp.Pose([0, 0, 0.7], [0, 0, -np.pi/2]),
        #     pp.Pose([0, 0, 0.6], [0, 0, -np.pi/2]),
        #     pp.Pose([0, 0.05, 0.6], [0, 0, -np.pi/2]),
        #     pp.Pose([0, 0.1, 0.55], [0, 0, -np.pi/2]),
        #     pp.Pose([0, 0, 0.5], [0, 0, -np.pi/2])
        # ]
        # print(waypoints)

        # plan = pp.plan_waypoints_joint_motion(self.sim_instance.sim_robot, [0, 2, 3, 4, 5], waypoints=wp)

        # print(plan)


class CmdGenerateNormals(Command):

    def __init__(self, controller, timeout=0):
        super().__init__(controller, timeout)

    def executeCommand(self, *args):
        if super(CmdGenerateNormals, self).executeCommand(args):
            print("[CMD] Generating object normals!")

    def onUpdate(self):
        super(CmdGenerateNormals, self).onUpdate()
        # self.sim_instance.current_point_cloud = simhelper.get_surface_alignment_points(draw_cloud=False)
        self.sim_instance.current_point_cloud = simhelper.get_grouped_alignment_points()
        self.complete = True


class CmdRestartSim(Command):

    def __init__(self, controller, timeout=0):
        super().__init__(controller, timeout)

    def executeCommand(self, args):
        super(CmdRestartSim, self).executeCommand(args)
        print("[CMD] Restarting Simulation!")

    def onUpdate(self):
        super(CmdRestartSim, self).onUpdate()

        self.sim_instance.can_run = False
        p.resetSimulation()
        p.disconnect()
        time.sleep(2)
        self.sim_instance.initialize_sim()
        self.controller_instance.current_command = None  # had to bypass the complete flag due to the uniqueness of cmd
        self.sim_instance.can_run = True


class CmdGoToPosiiton(Command):

    def __init__(self, controller, timeout=0):
        super().__init__(controller, timeout)

    def executeCommand(self, args):
        if super(CmdGoToPosiiton, self).executeCommand(args):
            print("[CMD] Going to Position!")

    def onUpdate(self):
        super().onUpdate()

        if self.args is None and len(self.args) != 1:
            print("[CMD] Invalid arguments given to command!")
            self.complete = True

        goal_pos = self.args[0][0]
        joint_poses = p.calculateInverseKinematics(self.sim_instance.sim_robot, 6, goal_pos,
                                                   p.getQuaternionFromEuler([0, 0, -np.pi / 2]))
        self.enableMotor(joint_poses, [0, 2, 3, 4, 5])

        if np.isclose(joint_poses, self.sim_instance.get_sim_joint_states(), atol=0.001).all():
            self.complete = True

    def enableMotor(self, joint_pos, joints):
        # awful code but no better solution atm

        # RAIL
        p.setJointMotorControl2(self.sim_instance.sim_robot, joints[0],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[0],
                                force=1000,
                                positionGain=0.5,
                                velocityGain=1,
                                maxVelocity=0.125)

        # WAIST
        p.setJointMotorControl2(self.sim_instance.sim_robot, joints[1],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[1],
                                force=1000,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=0)

        # SHOULDER
        p.setJointMotorControl2(self.sim_instance.sim_robot, joints[2],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[2],
                                force=1000,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=1.5)

        # ELBOW
        p.setJointMotorControl2(self.sim_instance.sim_robot, joints[3],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[3],
                                force=1000,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=1.5)

        # WRIST/PROBE
        p.setJointMotorControl2(self.sim_instance.sim_robot, joints[4],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[4],
                                force=1000,
                                positionGain=1,
                                velocityGain=1,
                                maxVelocity=2)
