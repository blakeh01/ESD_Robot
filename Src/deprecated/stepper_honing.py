step = input("Type step for stepping mode, write for writing mode.")

if step == "step":
    checker = input("Enter to step. 'f' to exit.")

    while checker != 'f':
        print(f"Writing {self.generate_g_code(self.current_pos)} to motor!")
        self.stepper_serial.write(self.generate_g_code(self.current_pos))
        self.current_pos += 30
        checker = input("Enter to step. 'f' to exit")
elif step == "write":
    rot = input("Enter a rotation to write. (- moves left, + moves right) [f to exit]: ")

    while rot != 'f':
        print(f"Writing {self.generate_g_code(int(rot))} to motor!")
        self.stepper_serial.write(self.generate_g_code(int(rot)))
        self.current_pos = int(rot)
        rot = input("Enter a rotation to write. (- moves left, + moves right) [f to exit]: ")

exit()

# MIRROR CODe

if self.mode == MODE_MIRROR:
    val = p.readUserDebugParameter(self.button_id)

    if val % 2 == 0:
        state = self.robot_instance.get_rbt_joint_states()
        p.setJointMotorControlArray(self.simulation_instance.sim_robot_id, [0, 2, 3, 4, 5],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=state,
                                    forces=[5000, 5000, 5000, 5000, 5000],
                                    positionGains=np.ones(5),
                                    velocityGains=np.ones(5))
    else:
        state = self.simulation_instance.get_sim_joint_states()
        self.robot_instance.sync_write_states(state)

# motor code
p.setJointMotorControlArray(self.simulation_instance.sim_robot_id, [0, 2, 3, 4, 5],
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=joint_poses,
                            forces=[1000, 1000, 1000, 1000, 1000],
                            positionGains=[0.1, 0.1, 0.1, 0.1, 0.1],
                            velocityGains=[0.5, 0.5, 0.5, 0.5, 0.5])
