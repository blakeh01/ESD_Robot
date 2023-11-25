"""

    Object profiles is a solution to a problem with not being able to follow the same execution flow for
    probing a complex object.

    Rotational symmetric objects can be processed like such:
        - slice upon the z-axis
        - move probe to top most z-axis point, closest to the robot
        - revolve the platform, since the object is rotationally symmetirc, it will hit all points on Z-axis.
        - move down to next slice until some arbitrary z-height to avoid collision

    Rectangular objects can be processed like such:
        - slice and group on probe normals to group faces
        - sweep across a face
        - move probe back, rotate object
        - repeat

    Other objects ???
        - Would be cool to have a system for users to extend functionality with a script..?
"""

import matplotlib.pyplot as plt
from Src.sim.Command import *
from Src.sim.simhelper import *
from Src.sim.simulation import Simulation
from Src.util.math_util import *

from Src.robot.SerialMonitor import *

MOVE_TO_POINT = 0
MEASURE = 1
GROUND = 2


class ObjectProfileUpdated:

    def __init__(self, simulation: Simulation, flow_list, flow_args):
        self.sim = simulation
        self.probe_points = simulation.normal_point_cloud.alignment_points
        self.robot = simulation.robot_handler
        self.flow_list = flow_list  # list of actions (probe/discharge/charge/wait)

        self.cur_flow_idx = 0
        self.cur_flow = None
        self.can_run = True

        # retrieve flow arguments
        self.rbt_max_speed = float(flow_args[0])  # in mm/s
        self.rotator_feedrate = float(flow_args[1])  # in mm/s
        self.grounding_interval = float(flow_args[2])  # in ms
        self.measuring_time = float(flow_args[3])  # convert to ms

        self.probe_action_state = None
        self.action_timeout = 0
        self.probe_percentage = 0

    def update(self, time_elapsed):

        if self.cur_flow_idx < len(self.flow_list):
            self.cur_flow = self.flow_list[self.cur_flow_idx]
        else:
            self.can_run = False
            print("Flow completed!")

        ##################### FLOW STATE MACHINE #####################

        ### WAIT ###
        if isinstance(self.cur_flow, Wait):
            if self.cur_flow.end_time == 0 and self.cur_flow.start_time == 0:
                self.cur_flow.start_time = time_elapsed
                self.cur_flow.end_time = self.cur_flow.start_time + self.cur_flow.wait_time  # sent end time to current time + wait amount

            if time_elapsed >= self.cur_flow.end_time:  # if current time is greater than wait time, then this is complete!
                print("==> Wait complete!")
                self.cur_flow_idx += 1  # action complete, advance flow index to retrieve next action.

        ### CHARGE ###
        elif isinstance(self.cur_flow, Charge):
            if self.cur_flow.start_time == 0:  # if this is the first time this is called, set the btn and label to visible for the user.
                self.cur_flow.start_time = time_elapsed
                self.action_timeout = time_elapsed + 0.25  # wait a quarter second
                self.sim.parent.btn_charge_done.setVisible(True)  # set 'charge done' button to visible
                self.sim.parent.lbl_charge_warn.setVisible(True)  # set 'charge' label to visible.

            if time_elapsed >= self.action_timeout:  # flash the background of the charge label to give more urgency.
                self.action_timeout = time_elapsed + 0.25
                self.sim.parent.lbl_charge_warn.setStyleSheet(
                    "background-color: lightgreen" if self.sim.parent.lbl_charge_warn.styleSheet() == "background-color: white" else "background-color: white")

            if self.cur_flow.is_charged:  # if the charge done flag is set by something, consider this action complete!
                print("==> Charge Complete!")
                self.cur_flow.end_time = time_elapsed
                self.cur_flow_idx += 1  # advance to next action

        ### DISCHARGE ###
        elif isinstance(self.cur_flow, Discharge):  # if the action is discharge
            if self.cur_flow.start_time == 0:
                self.cur_flow.start_time = time_elapsed
                self.cur_flow.discharge()  # this is currently blocking, should not matter, but maybe make unblocking
                self.cur_flow_idx += 1

        ### PROBE ###
        elif isinstance(self.cur_flow, Probe):

            if self.cur_flow.start_time == 0:
                self.cur_flow.start_time = time_elapsed
                self.probe_action_state = GROUND  # force to ground at first

            ### MOVE RBT AND PLATFORM TO NEXT POINT ###

            if self.probe_action_state == MOVE_TO_POINT:
                rbt_pos, plat_rot = self.get_next_probe_point()

                ### Check if no more moves remaining, therefore we are done ###
                if rbt_pos is None or plat_rot is None:
                    self.cur_flow.end_time = time_elapsed
                    print("Probing Complete!")

    def construct_probe_plan(self):
        """

        @return:
        """
        pass

    def get_next_probe_point(self):
        pass


class RectangularPrism(ObjectProfileUpdated):

    def __init__(self):
        pass

    def update(self):
        super(RectangularPrism, self).update()


class ObjectProfile:

    def __init__(self, simulation: Simulation, flow, flow_args):
        """
        An object profile is the heart of the probing algorithm. The point of the object profiles is to characterize
        each type of object for scanning, thereby allowing for the quickest scan possible.

        The downside of this system is that each object needs to be characterized (i.e. knowing that it is
        rotationally symmetric) prior to creating an object profile. Two main profiles are currently supported:

        * Rectangular geometry - characterized by having N-faces that can be subdivided
        * Rotationally symmetric geometry - characterized by having a singular rotational axis (along Z - up/down)

        @param simulation: An instance to the current simulation for retrival of the current normal point cloud
        @param flow: A list of action keywords (Charge, Discharge, Probe, Wait) that are associated with this profile.
        @param flow_args: A list of user entered arguments (i.e. grounding interval and max speed)
        """
        self.sim = simulation
        self.probe_points = simulation.normal_point_cloud.alignment_points
        self.robot = simulation.robot_handler
        self.flow = flow

        self.cur_flow_idx = 0
        self.probe_percentage = 0
        self.can_run = True
        self.charge_done_flag = False

        # retrieve flow arguments
        self.rbt_max_speed = float(flow_args[0])
        self.rotator_feedrate = float(flow_args[1])
        self.grounding_interval = float(flow_args[2])
        self.measuring_time = float(flow_args[3]) / 1000  # convert to ms

        self.ground_flag = False  # set to true when robot should ground after completed movement

    def initialize(self):
        """
        Initialization of an object profile (ie. RotationallySymmetric or Rectangular) requires that all points
        in the point cloud are processed and ready for probing and visualization.
        """
        pass

    def update(self, time_elapsed):
        """
        Updating the probe flow runs the current profile loaded, as long as one exists and is not completed.
        If a probe flow is completed, it is marked as complete and self.can_run is set to FALSE.

        Otherwise, the probe flow is executed starting with the first action in the queue. Each action, (Charge, Discharge, Wait, Probe)
        has an associated start time, end time, and other parameters depending on the action.

        @param time_elapsed: Current time elapsed by the simulation.
        """
        pass


class RotationallySymmetric(ObjectProfile):
    """
        Rotationally Symmetric object profile encapsulates the probing algorithm behind rotationally symmetric objects.
        These objects have a very specific scan pattern that ensures max efficiency, and therefore quickens probing.

        Rotationally symmetric objects are processed by creating slices along the Z-axis (up/down), then processing
        those slices from top to bottom. Each slice is processed by taking the current probe position, finding the
        nearest point from the probe tip to any point along the slice, then computing the least cost path by solving
        the travelling salesman problem.
    """

    def __init__(self, simulation: Simulation, flow, flow_args):
        super(RotationallySymmetric, self).__init__(simulation, flow, flow_args)

        self.z_slices = {}  # a list of slices made along the Z-axis for probing
        self.z_sil_index = 0  # the current index of Z-slice being probed
        self.cur_slice = None  # a reference to the current slice (this is z_slices[] @ index z_sil_index)
        self.tolerance = 0.005  # a tolerance variable for grouping of Z-axis coordinates during slicing
        self.min_points_slice = 5  # a minimum number of points per slice before throwing an error.
        self.visualize = True  # a flag to show a matplotlib graph of the generated slices.

        self.cur_path = None  # a list of points (of the cur_slice) that are sorted via a least cost pathing function.
        self.cur_point_index = 0  # the current point we are probing from the current path.

        self.next_ground_time = 0  # a reference variable to find the next ground time (usually time_elapsed + grouding_interval)

        self.action_wait_end = 0  # a generic action reference variable to create delay between necessary components.

        self.measure_flag = False  # a flag that is raised when the robot has reached the point and is ready to measure.

        self.initialize()

    def initialize(self):
        print("Grouping probe points by Z-axis with tolerance of ", self.tolerance, " m!")

        # group points by Z with a tolerance value
        for point in self.probe_points:
            z_value = round(point.pos[2], 3)  # Round to 3 decimal places for tolerance

            # iterate through keys in self.z_slices, store each unique z-value as a key to a dictionary
            # put any similar z-axis points in the same dictionary key
            for existing_z, points_list in self.z_slices.items():
                if abs(existing_z - z_value) <= self.tolerance:
                    points_list.append(point)  # found that key w/ z-axis exists, so add it to the dictionary.
                    break
            else:
                self.z_slices[z_value] = [point]  # create new key and store the current point

        # search for malformed slices, try to assign them to other slices.
        # if this happens, its relatively a 'bad' thing, so try to warn the user just in case.

        # this is needed as the precision of the normal generate is finite, therefore there could be points from the
        # normal generation that randomly fall between 2 slices, with no apparent group
        # if this happens, then we assign it to the closes slice, but we warn beacuse if the deviation is significant,
        # the time to measure this point will be costly.
        for z_value, points_list in self.z_slices.items():
            if len(points_list) < self.min_points_slice:
                print(
                    "Detected a malformed slice! Try decreasing the tolerance of the object profile or decrease probing resolution!")
                # Find the neighboring slice with the most points
                neighboring_slices = [z for z in self.z_slices.keys() if
                                      abs(z - z_value) <= self.tolerance and z != z_value]
                if neighboring_slices:
                    neighboring_slices.sort(key=lambda z: len(self.z_slices[z]), reverse=True)
                    target_slice = neighboring_slices[0]
                    self.z_slices[target_slice].extend(points_list)
                    del self.z_slices[z_value]

        # visualize slices using matplotlib
        if self.visualize:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            # Create a list of colors for visualization
            colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

            # Scatter plot, chose color based on index
            for i, (z_value, points) in enumerate(self.z_slices.items()):
                z_color = colors[i % len(colors)]
                z_values = [point.pos[0] for point in points]
                y_values = [point.pos[1] for point in points]
                x_values = [point.pos[2] for point in points]
                ax.scatter(x_values, y_values, z_values, c=z_color, label=f'Z Value: {z_value}')

            # Set labels
            ax.set_xlabel('X-axis')
            ax.set_ylabel('Y-axis')
            ax.set_zlabel('Z-axis')

            # Set title
            plt.title('Slices Visualization')

            # Show the plot
            plt.show()

        self.z_slices = sort_and_convert_to_list(self.z_slices)
        print("Generated " + str(len(self.z_slices)) + " slices!")

    def update(self, time_elapsed):
        if not self.can_run:
            return

        # if we have reached the end of the flow, then set can_run to false and notify user.
        if self.cur_flow_idx + 1 > len(self.flow):
            print("Probe flow completed!")
            self.can_run = False
            return

        cur_flow = self.flow[
            self.cur_flow_idx]  # set the current action using the current flow index (which is incremented after an action is complete)

        if isinstance(cur_flow, Wait):  # if the current action is to wait
            if cur_flow.end_time == 0 and cur_flow.start_time == 0:
                cur_flow.start_time = time_elapsed
                cur_flow.end_time = cur_flow.start_time + cur_flow.wait_time  # sent end time to current time + wait amount

            if time_elapsed >= cur_flow.end_time:  # if current time is greater than wait time, then this is complete!
                print("Waiting completed... moving to next step!")
                self.cur_flow_idx += 1  # action complete, advance flow index to retrieve next action.

        elif isinstance(cur_flow, Charge):  # if the current action is charge
            if cur_flow.start_time == 0:  # if this is the first time this is called, set the btn and label to visible for the user.
                cur_flow.start_time = time_elapsed
                self.action_wait_end = time_elapsed + 0.25  # wait a quarter second
                self.sim.parent.btn_charge_done.setVisible(True)  # set 'charge done' button to visible
                self.sim.parent.lbl_charge_warn.setVisible(True)  # set 'charge' label to visible.
            else:
                if time_elapsed >= self.action_wait_end:  # flash the background of the charge label to give more urgency.
                    self.action_wait_end = time_elapsed + 0.25
                    self.sim.parent.lbl_charge_warn.setStyleSheet(
                        "background-color: lightgreen" if self.sim.parent.lbl_charge_warn.styleSheet() == "background-color: white" else "background-color: white")

            if self.charge_done_flag:  # if the charge done flag is set by something, consider this action complete!
                cur_flow.end_time = time_elapsed
                print("[USER] Charge Complete!")
                self.cur_flow_idx += 1  # advance to next action
                self.charge_done_flag = False  # reset charge done flag

        elif isinstance(cur_flow, Discharge):  # if the action is discharge (TODO: CLEAN THIS UP)
            if cur_flow.start_time == 0:
                cur_flow.start_time = time_elapsed
                cur_flow.set_stepper(
                    self.sim.robot_handler.stepper_board)  # set the stepper board to the one instatiated at start of program
                cur_flow.discharge()  # call the discharge function (todo: make LDS/stepper static instance in robot)
                self.cur_flow_idx += 1  # advance flow when done (code halts with above statement, shouldn't be an issue)

        elif isinstance(cur_flow, Probe):  # if the action is probe

            # this is the BIG one:

            if cur_flow.start_time == 0:  # at the start of the action
                cur_flow.start_time = time_elapsed
                self.z_sil_index = 0  # set z slice index to 0, so we are starting at the top (note that z-slices was sorted in the initialization)
                self.probe_percentage = 0  # set probing percentage to 0
                print("START PROBE!")
                self.ground_flag = True  # set our ground flag to be true s.t. it grounds at the beginning
            else:

                # if the current z slice index is GREATER THAN (or EQ) than the number of slices, we have computed all slices.
                # mark as complete, advance flow, and drive the mootors home.
                if self.z_sil_index >= len(self.z_slices):
                    print("Probing Completed!")
                    cur_flow.end_time = time_elapsed
                    self.cur_flow_idx += 1

                    # hacky way to quickly step simulation to move the motors home
                    # this is purely a quirk of pybullet.
                    for i in range(100):
                        self.sim.drive_motors_to_home()
                        p.stepSimulation()
                        time.sleep(1 / 120)

                    # after the simulation robot has been driven home, then set goal conf of robot to those home states.
                    self.sim.robot_handler.set_goal_conf(
                        pp.get_joint_positions(self.sim.sim_robot, [1, 2, 3, 4, 5]))

                    return  # exit as we have completed, no need to do further checks.

                # if the current slice is NULL (which happens either at the start, or after a slice is complete)
                # get the slice at z-slice-index and start doing magic
                if not self.cur_slice:
                    self.cur_slice = self.z_slices[self.z_sil_index]
                    print("Starting probe on Z-slice: ", self.z_sil_index, " Z-val: ", self.cur_slice[0])

                    print("Selecting point closest to robot!")
                    closest = find_closest_point(self.cur_slice[1], pp.get_link_pose(self.sim.sim_robot, 6)[
                        0])  # in the slice (which contains a bunch of points) find the nearest point from the probe to any point to determine a start location.

                    self.cur_path = find_alignment_point_path(closest, self.cur_slice[
                        1])  # using closest point, find least cost path and set that to our current path
                    self.sim.parent.plot_slice(self.cur_slice[1],
                                               self.cur_path)  # plot this slice on the GUI for the user
                    print("Completed plotting slices! Beginning movements!")
                else:

                    # if the current point index is greater than the lenght of the path, we have reached the end of the path and therefore have scanned the slice.
                    if self.cur_point_index >= len(self.cur_path):
                        print("Reached end of slice!")
                        self.z_sil_index += 1  # increment z slice index to go to the next slice
                        self.cur_point_index = 0  # set current point index back to zero
                        self.cur_slice = None  # and set our current slice back to none to recalculate next slice.
                        return  # return as we do not wan to do anything else after slice completion

                    # if the platform and robot have finished their movement, and there is no ground flag or measure flag,
                    # rotate the platform to line up point and move robot any incremental amount.
                    if self.sim.pos_plat_command.complete and self.sim.pos_probe_command.complete and not self.ground_flag and not self.measure_flag:
                        # update labels for the current slice for user feedback
                        self.probe_percentage = int(100 * (self.cur_point_index / (len(self.cur_path) - 1)))
                        self.sim.parent.lbl_slice_index.setText(str(self.z_sil_index))
                        self.sim.parent.lbl_point_index.setText(str(self.cur_point_index))

                        # if next ground time has NOT been set... SET IT!
                        if self.next_ground_time == 0:
                            self.next_ground_time = self.grounding_interval + time_elapsed  # setting grounding time to the current time + grounding interval
                            print("Setting next ground time to: ", self.next_ground_time)

                        print("Processing slice i: ", self.z_sil_index, " | Current point index: ",
                              self.cur_point_index)

                        # get point from current index
                        pt = self.cur_path[self.cur_point_index]  # get the current point

                        # find angle between point and lineup vector (lineup vector is the vector that points from the center of the rotating platform to the robot)
                        # todo: diagram for this?
                        angle = np.math.atan2(np.linalg.det([pt.direction[0:2], self.sim.lineup_normal[0:2]]),
                                              np.dot(pt.direction[0:2], self.sim.lineup_normal[0:2]))

                        # call command to rotate platform to angle so that the point will be lined up
                        self.sim.pos_plat_command = PlatformPositionSetter(self.sim, angle)

                        # As we have rotated our object, we must update the point cloud using a rotation matrix
                        temp = []
                        for i in range(len(self.cur_path)):
                            pos = np.dot(rotation_matrix_z(angle), self.cur_path[i].pos)
                            dir = np.dot(rotation_matrix_z(angle), self.cur_path[i].direction)
                            temp.append(AlignmentPoint(pos, dir))

                        self.cur_path = temp

                        # set the probe position to the newly rotated point.
                        self.sim.pos_probe_command = ProbePositionSetter(self.sim,
                                                                         self.cur_path[self.cur_point_index].pos,
                                                                         [0, 0, 0]
                                                                         )
                        self.measure_flag = True  # after these two positions have been queued, we can now measure when they arrive!

                    # if our measure flag has been set, and the movement has been completed --> TAKE MEASUREMENT
                    if self.measure_flag and self.sim.pos_plat_command.complete and self.sim.pos_probe_command.complete and not self.ground_flag:
                        # if we have a finite measuring time, wait that amount of time before measuring, otherwise just measure
                        if self.measuring_time > 0:
                            if self.action_wait_end == 0:
                                self.action_wait_end = time_elapsed + self.measuring_time
                            elif time_elapsed >= self.action_wait_end:
                                print("[FIX] BEEP PROBE VOLTAGE!")
                                self.cur_path[self.cur_point_index].measurement = self.sim.parent.probe_voltage
                                self.cur_point_index += 1
                                self.measure_flag = False
                                self.action_wait_end = 0
                        else:
                            print("[FIX] BEEP PROBE VOLTAGE!")
                            self.cur_path[self.cur_point_index].measurement = self.sim.parent.probe_voltage
                            self.cur_point_index += 1
                            self.measure_flag = False

        # if the next grounding time is less than time elapsed (i.e. its time to ground), then raise ground flag!
        if self.next_ground_time <= time_elapsed and self.next_ground_time != 0:
            print("Time to ground! Raising flag!")
            self.ground_flag = True
            self.next_ground_time = 0

        # if there are no current movements (i.e. movement is complete) AND the ground flag is set:
        # Offset the current probe position by a small amount such that it backs away from the object
        # write to our action timer to wait 4 seconds, and send a serial comm to the feather telling it to ground
        if self.ground_flag and self.sim.pos_probe_command.complete and self.sim.pos_plat_command.complete and self.action_wait_end == 0:
            new_point = pp.get_link_pose(self.sim.sim_robot, 6)[0]

            new_point = np.add(new_point, [-.075, 0, 0])  # offset probe
            self.sim.pos_probe_command = ProbePositionSetter(self.sim, new_point, [0, 0, 0])  # todo get joint orn?
            self.action_wait_end = time_elapsed + 4  # wait 4 seconds to let system ground
            self.sim.robot_handler.feather0.write_data(b'1')  # tell servo to ground!

        # after our grounding action, reset ground flag and action wait timer.
        if self.action_wait_end != 0 and time_elapsed >= self.action_wait_end and self.ground_flag:
            self.action_wait_end = 0
            self.ground_flag = False


class RectangularPrisms(ObjectProfile):
    """
        Rectangular Object Profile is one that has N-faces that can be discretely grouped and pathed between.

    """

    def __init__(self, simulation: Simulation, flow, flow_args):
        super(RectangularPrisms, self).__init__(simulation, flow, flow_args)

        self.normal_slices = {}
        self.tolerance = 0.9
        self.min_points = 5

        self.cur_path = None
        self.cur_slice = None
        self.cur_point_index = 0
        self.side_index = 0

        self.ground_flag = False
        self.next_ground_time = 0

        self.measure_flag = False

        self.action_wait_start = 0
        self.action_wait_end = 0

        self.initialize()

    def initialize(self):
        def calculate_cosine_similarity(vec1, vec2):
            return np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))

        self.normal_slices = {}

        for point in self.probe_points:
            direction = point.direction
            added_to_existing_slice = False

            for existing_dir, points_list in self.normal_slices.items():
                similarity = calculate_cosine_similarity(direction, existing_dir)
                if similarity >= self.tolerance:
                    points_list.append(point)
                    added_to_existing_slice = True
                    break

            if not added_to_existing_slice:
                self.normal_slices[tuple(direction)] = [point]

        for direction, points_list in self.normal_slices.items():
            if len(points_list) < self.min_points:
                closest_slice = None
                closest_similarity = 0

                for existing_dir, existing_points in self.normal_slices.items():
                    if existing_dir != direction:
                        similarity = calculate_cosine_similarity(direction, existing_dir)
                        if similarity > closest_similarity:
                            closest_similarity = similarity
                            closest_slice = existing_dir

                if closest_slice is not None:
                    self.normal_slices[closest_slice].extend(points_list)
                    del self.normal_slices[direction]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

        for i, (direction, points) in enumerate(self.normal_slices.items()):
            color = colors[i % len(colors)]
            x_values = [point.pos[0] for point in points]
            y_values = [point.pos[1] for point in points]
            z_values = [point.pos[2] for point in points]
            ax.scatter(x_values, y_values, z_values, c=color, label=f'Group {i}')

        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        plt.title('Slices Visualization')
        ax.legend(loc='upper right')
        plt.show()

        # Sort slices, starting with side nearest to robot, then use a nearest neighbor algorithm to find a good path.
        point_list = list(self.normal_slices.keys())
        sorted = nearest_neighbor_dict_sort(point_list, find_nearest_point_index(self.normal_slices,
                                                                                 pp.get_link_pose(self.sim.sim_robot,
                                                                                                  6)[0]))
        self.normal_slices = {point: self.normal_slices[point] for point in sorted}

        self.normal_slices = [((x, y, z), data) for (x, y, z), data in self.normal_slices.items()]

        print(self.normal_slices)

    def update(self, time_elapsed):
        if not self.can_run:
            return

        if self.cur_flow_idx + 1 > len(self.flow):
            print("Probe flow completed!")
            self.can_run = False
            return

        cur_flow = self.flow[self.cur_flow_idx]

        if isinstance(cur_flow, Wait):
            if cur_flow.end_time == 0 and cur_flow.start_time == 0:
                cur_flow.start_time = time_elapsed
                cur_flow.end_time = cur_flow.start_time + cur_flow.wait_time

            if time_elapsed >= cur_flow.end_time:
                print("Waiting completed... moving to next step!")
                self.cur_flow_idx += 1
        elif isinstance(cur_flow, Charge):
            if cur_flow.start_time == 0:
                cur_flow.start_time = time_elapsed
                self.action_wait_end = time_elapsed + 0.25
                self.sim.parent.btn_charge_done.setVisible(True)
                self.sim.parent.lbl_charge_warn.setVisible(True)
            else:
                if time_elapsed >= self.action_wait_end:
                    self.action_wait_end = time_elapsed + 0.25
                    self.sim.parent.lbl_charge_warn.setStyleSheet(
                        "background-color: lightgreen" if self.sim.parent.lbl_charge_warn.styleSheet() == "background-color: white" else "background-color: white")

            if self.charge_done_flag:
                cur_flow.end_time = time_elapsed
                print("[USER] Charge Complete!")
                self.cur_flow_idx += 1
                self.charge_done_flag = False

        elif isinstance(cur_flow, Discharge):
            if cur_flow.start_time == 0:
                cur_flow.start_time = time_elapsed
                cur_flow.set_stepper(self.sim.robot_handler.stepper_board)
                cur_flow.discharge()
                self.cur_flow_idx += 1
        elif isinstance(cur_flow, Probe):

            if cur_flow.start_time == 0:
                cur_flow.start_time = time_elapsed
                self.side_index = 0
                print("PROBING! Sending robot home...")
                self.ground_flag = True
            else:

                if cur_flow.start_time == 0:
                    cur_flow.start_time = time_elapsed
                    self.z_sil_index = 0
                    self.probe_percentage = 0
                    print("START PROBE!")
                    self.ground_flag = True
                else:

                    if self.side_index >= len(self.normal_slices):
                        print("Probing Completed!")
                        cur_flow.end_time = time_elapsed
                        self.cur_flow_idx += 1

                        for i in range(100):
                            self.sim.drive_motors_to_home()
                            p.stepSimulation()
                            time.sleep(1 / 120)

                        self.sim.robot_handler.set_goal_conf(
                            pp.get_joint_positions(self.sim.sim_robot, [1, 2, 3, 4, 5]))

                        return

                if not self.cur_slice:
                    self.cur_slice = self.normal_slices[self.side_index]
                    print("Starting probing on face i: ", self.side_index, " Side Position: ", self.cur_slice[0])

                    optimal = find_corner(self.cur_slice[1])
                    self.cur_path = find_alignment_point_path(optimal, self.cur_slice[1])

                    # self.sim.parent.plot_slice(self.cur_slice[1], self.cur_path)
                    print("Completed plotting slices! Beginning movements!")

                if self.cur_point_index >= len(self.cur_path):
                    print("Reached end of slice!")
                    self.side_index += 1
                    self.cur_point_index = 0
                    self.cur_slice = None

                    new_point = pp.get_link_pose(self.sim.sim_robot, 6)[0]
                    new_point = np.add(new_point, [-.075, 0, 0])  # offset probe
                    self.sim.pos_probe_command = ProbePositionSetter(self.sim, new_point,
                                                                     [0, 0, 0])  # todo get joint orn?
                    return

                if self.sim.pos_plat_command.complete and self.sim.pos_probe_command.complete and not self.ground_flag and not self.measure_flag:
                    self.probe_percentage = int(100 * (self.cur_point_index / (len(self.cur_path) - 1)))
                    self.sim.parent.lbl_slice_index.setText(str(self.side_index))
                    self.sim.parent.lbl_point_index.setText(str(self.cur_point_index))

                    if self.next_ground_time == 0:
                        self.next_ground_time = self.grounding_interval + time_elapsed
                        print("Setting next ground time to: ", self.next_ground_time)

                    print("Processing slice i: ", self.side_index, " | Current point index: ",
                          self.cur_point_index)
                    # get point from current index
                    pt = self.cur_path[self.cur_point_index]

                    # find angle between point and lineup vector
                    angle = np.math.atan2(np.linalg.det([pt.direction[0:2], self.sim.lineup_normal[0:2]]),
                                          np.dot(pt.direction[0:2], self.sim.lineup_normal[0:2]))

                    # rotate platform to angle
                    self.sim.pos_plat_command = PlatformPositionSetter(self.sim, angle)

                    # rotate cur path
                    temp = []
                    for i in range(len(self.cur_path)):
                        pos = np.dot(rotation_matrix_z(angle), self.cur_path[i].pos)
                        dir = np.dot(rotation_matrix_z(angle), self.cur_path[i].direction)
                        temp.append(AlignmentPoint(pos, dir))

                    self.cur_path = temp
                    self.sim.pos_probe_command = ProbePositionSetter(self.sim, self.cur_path[self.cur_point_index].pos,
                                                                     goal_orn=[0, 0, 0])
                    self.measure_flag = True

                if self.measure_flag and self.sim.pos_plat_command.complete and self.sim.pos_probe_command.complete and not self.ground_flag:
                    if self.measuring_time > 0:
                        if self.action_wait_end == 0:
                            self.action_wait_end = time_elapsed + self.measuring_time
                        elif time_elapsed >= self.action_wait_end:
                            print("[FIX] BEEP PROBE VOLTAGE!")
                            self.cur_path[self.cur_point_index].measurement = 10  # self.sim.parent.probe_voltage
                            self.cur_point_index += 1
                            self.measure_flag = False
                            self.action_wait_end = 0
                    else:
                        print("[FIX] BEEP PROBE VOLTAGE!")
                        self.cur_path[self.cur_point_index].measurement = 10  # self.sim.parent.probe_voltage
                        self.cur_point_index += 1
                        self.measure_flag = False

        if self.next_ground_time <= time_elapsed and self.next_ground_time != 0:
            print("Time to ground! Raising flag!")
            self.ground_flag = True
            self.next_ground_time = 0

        if self.ground_flag and self.sim.pos_probe_command.complete and self.sim.pos_plat_command.complete and self.action_wait_end == 0:
            new_point = pp.get_link_pose(self.sim.sim_robot, 6)[0]

            new_point = np.add(new_point, [-.075, 0, 0])  # offset probe
            self.sim.pos_probe_command = ProbePositionSetter(self.sim, new_point, [0, 0, 0])  # todo get joint orn?
            self.action_wait_end = time_elapsed + 4  # wait 4 seconds to let system ground
            self.sim.robot_handler.feather0.write_data(b'1')  # tell servo to ground!

        if self.action_wait_end != 0 and time_elapsed >= self.action_wait_end and self.ground_flag:
            self.action_wait_end = 0
            self.ground_flag = False


class Charge:

    def __init__(self):
        self.start_time = 0
        self.end_time = 0

        self.is_charged = False


class Discharge:

    def __init__(self, stepper_inst, lds_inst, obj_z):
        self.start_time = 0
        self.end_time = 0

        self.x_feed = 1500
        self.y_feed = 500  # This needs to be changed to the equivalent of 50mm/s
        self.z_feed = 500
        self.step = 1

        self.complete = False

        self.stepper_board = stepper_inst
        self.LDS = lds_inst

        self.obj_z = obj_z

    def discharge(self):
        self.stepper_board.home_scan()
        time.sleep(15)

        self.stepper_board.write_z((self.obj_z / 2) - 25, self.z_feed)
        self.stepper_board.write_x(102, self.x_feed)
        time.sleep(15)

        y = self.LDS.read_distance() + 10000
        y = int(y / 100)

        # write to z to half object height (-25 to account for CVR)
        # write to x to center (102)

        print("writing to y: ", y)

        if y > 175 or y < 0:
            return

        self.stepper_board.write_y(y - 30, self.y_feed)
        self.stepper_board.read_data()
        time.sleep(10)

        self.stepper_board.home_scan()


class Probe:

    def __init__(self):
        self.start_time = 0
        self.end_time = 0


class Wait:

    def __init__(self, wait_time):
        self.wait_time = wait_time

        self.start_time = 0
        self.end_time = 0


def sort_and_convert_to_list(z_slices):
    sorted_slices = sorted(z_slices.items(), key=lambda item: item[0], reverse=True)
    return sorted_slices
