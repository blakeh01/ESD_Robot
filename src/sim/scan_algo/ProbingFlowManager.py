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
from src.sim.Command import *
from src.sim.simhelper import *
from src.sim.simulation import Simulation
from src.util.math_util import *

from src.robot.SerialMonitor import *

MOVE_TO_POINT = 0
MEASURE = 1


class ProbingFlowManager:

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
        self.ground_flag = False
        self.next_ground_time = 0
        self.action_timeout = 0
        self.probe_percentage = 0

        self.cur_alignment_point = None
        self.goal_rbt_pos = None
        self.goal_rbt_orn = None
        self.goal_plat_rot = None

    def update(self, time_elapsed):
        if not self.can_run:
            return

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
                self.sim.controller.btn_charge_done.setVisible(True)  # set 'charge done' button to visible
                self.sim.controller.lbl_charge_warn.setVisible(True)  # set 'charge' label to visible.

            if time_elapsed >= self.action_timeout:  # flash the background of the charge label to give more urgency.
                self.action_timeout = time_elapsed + 0.25
                self.sim.controller.lbl_charge_warn.setStyleSheet(
                    "background-color: lightgreen" if self.sim.controller.lbl_charge_warn.styleSheet() == "background-color: white" else "background-color: white")

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
                self.ground_flag = True  # force to ground at first
                print("==> Probe Begin!")

            ### MOVE RBT AND PLATFORM TO NEXT POINT ###

            if self.probe_action_state == MOVE_TO_POINT and not self.ground_flag:
                # get next conf
                (self.goal_rbt_pos, self.goal_rbt_orn), self.goal_plat_rot = self.get_next_confs()

                ### Check if no more moves remaining, therefore we are done ###
                if self.goal_rbt_pos is None or self.goal_plat_rot is None:
                    self.cur_flow.end_time = time_elapsed
                    print("==> Probe Complete!")  # todo: add going home here
                    self.cur_flow_idx += 1
                    return

                print("Moving to next probe point!")

                # call command to rotate platform to angle so that the point will be lined up
                self.sim.pos_plat_command = PlatformPositionSetter(self.sim, self.goal_plat_rot)

                # set the probe position to the newly rotated point.
                self.sim.pos_probe_command = ProbePositionSetter(self.sim,
                                                                 self.goal_rbt_pos,
                                                                 self.goal_rbt_orn
                                                                 )

                self.probe_action_state = MEASURE  # we have sent the positions, now wait and measure

            ### WAIT FOR MOVEMENT AND MEASURE ###
            elif self.probe_action_state == MEASURE:

                # exit out if we are still moving to the probe point
                if not self.sim.pos_probe_command.complete and not self.sim.pos_plat_command.complete:
                    return

                print("Movement complete, taking measurement!")

                # if measuring time is greater than 0, wait that time and then probe, otherwise just probe.
                if self.measuring_time > 0 and self.action_timeout == 0:
                    self.action_timeout = time_elapsed + self.measuring_time
                else:
                    self.cur_alignment_point.measurement = 1  # TODO: set to NIDAQ reading
                    self.probe_action_state = MOVE_TO_POINT  # now move to point once we are done measuring
                    return

                if time_elapsed >= self.action_timeout:
                    self.cur_alignment_point.measurement = 1  # TODO: set to NIDAQ reading
                    self.probe_action_state = MOVE_TO_POINT  # now move to point once we are done measuring
                    self.action_timeout = 0

            ### GROUND PROBE ###
            elif self.ground_flag and self.action_timeout == 0:
                print("Ground flag is raised! Grounding...")
                # offset probe by a static amount for backing off to allow room for ground probe
                new_point = pp.get_link_pose(self.sim.sim_robot, 6)[0]
                new_point = np.add(new_point, [-.075, 0, 0])
                self.sim.pos_probe_command = ProbePositionSetter(self.sim, new_point,
                                                                 [0, 0, 0])  # todo: match current joint orn?

                self.action_timeout = time_elapsed + 4  # start a 4-second timeout to finish ground action
                self.sim.robot_handler.feather0.write_data(b'1')  # writing serial 1 to feather is ground command

            ### CONDITIONAL STATE CHANGES ###
            if not self.ground_flag and time_elapsed >= self.next_ground_time:
                self.ground_flag = True
            elif self.ground_flag and time_elapsed >= self.action_timeout:
                self.ground_flag = False
                self.next_ground_time = time_elapsed + self.grounding_interval
                self.action_timeout = 0

    def construct_probe_plan(self):
        raise NotImplementedError("Please override this method for proper functionality!")

    def get_next_confs(self):
        raise NotImplementedError("Please override this method for proper functionality!")


class RotationallySymmetric(ProbingFlowManager):

    def __init__(self, simulation: Simulation, flow_list, flow_args):
        super().__init__(simulation, flow_list, flow_args)
        self.z_slices = {}  # a list of slices made along the Z-axis for probing
        self.z_sil_index = 0  # the current index of Z-slice being probed
        self.cur_slice = None  # a reference to the current slice (this is z_slices[] @ index z_sil_index)
        self.tolerance = 0.005  # a tolerance variable for grouping of Z-axis coordinates during slicing
        self.min_points_slice = 5  # a minimum number of points per slice before throwing an error.
        self.visualize = True  # a flag to show a matplotlib graph of the generated slices.

        self.cur_path = None  # a list of points (of the cur_slice) that are sorted via a least cost pathing function.
        self.cur_point_index = 0  # the current point we are probing from the current path.

        self.construct_probe_plan()

    def update(self, time_elapsed):
        super(RotationallySymmetric, self).update(time_elapsed)

    def construct_probe_plan(self):
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

    def get_next_confs(self):
        if self.z_sil_index >= len(self.z_slices):
            return None

        if not self.cur_slice:
            self.cur_slice = self.z_slices[self.z_sil_index]
            print("Starting probe on Z-slice: ", self.z_sil_index, " Z-val: ", self.cur_slice[0])

            print("Selecting point closest to robot!")
            # in the slice (which contains a bunch of points) find the nearest point from the probe to any point to
            # determine a start location.
            closest = find_closest_point(self.cur_slice[1], pp.get_link_pose(self.sim.sim_robot, 6)[0])

            # using the closest point, find the least cost path and set that to our current path
            self.cur_path = find_alignment_point_path(closest, self.cur_slice[1])
            self.sim.controller.plot_slice(self.cur_slice[1],
                                           self.cur_path)  # plot this slice on the GUI for the user
            print("Completed plotting slices! Beginning movements!")
        else:
            # if the current point index is greater than the length of the path, we have reached the end of the path
            # and therefore have scanned the slice.
            if self.cur_point_index >= len(self.cur_path):
                print("Reached end of slice!")
                self.z_sil_index += 1  # increment z slice index to go to the next slice
                self.cur_point_index = 0  # set current point index back to zero
                self.cur_slice = None  # and set our current slice back to none, so that we recalculate next slice.
                return  # return as we do not want to do anything else after slice completion

            # if the platform and robot have finished their movement, and there is no ground flag or measure flag,
            # rotate the platform to line up point and move robot any incremental amount.
            # update labels for the current slice for user feedback
            self.probe_percentage = int(100 * (self.cur_point_index / (len(self.cur_path) - 1)))
            self.sim.controller.lbl_slice_index.setText(str(self.z_sil_index))
            self.sim.controller.lbl_point_index.setText(str(self.cur_point_index))

            print("Processing slice i: ", self.z_sil_index, " | Current point index: ",
                  self.cur_point_index)

            # get point from current index
            pt = self.cur_path[self.cur_point_index]  # get the current point

            # find angle between point and lineup vector (lineup vector is the vector that points from the center of
            # the rotating platform to the robot) todo: diagram for this?
            goal_plat_rot = np.math.atan2(np.linalg.det([pt.direction[0:2], self.sim.lineup_normal[0:2]]),
                                          np.dot(pt.direction[0:2], self.sim.lineup_normal[0:2]))

            # As we have rotated our object, we must update the point cloud using a rotation matrix
            temp = []
            for i in range(len(self.cur_path)):
                pos = np.dot(rotation_matrix_z(goal_plat_rot), self.cur_path[i].pos)
                dir = np.dot(rotation_matrix_z(goal_plat_rot), self.cur_path[i].direction)
                temp.append(AlignmentPoint(pos, dir))

            pos, dir = self.cur_path[self.cur_point_index].pos, [0, 0, 0]  # todo rbt orn?

            self.cur_path = temp
            self.cur_point_index += 1

            return (pos, dir), goal_plat_rot

        return None


class RectangularPrism(ProbingFlowManager):
    """
        Rectangular Object Profile is one that has N-faces that can be discretely grouped and pathed between.

    """

    def __init__(self, simulation: Simulation, flow_list, flow_args):
        super().__init__(simulation, flow_list, flow_args)

        self.normal_slices = {}
        self.tolerance = 0.9
        self.min_points = 5

        self.cur_path = None
        self.cur_slice = None
        self.cur_point_index = 0
        self.side_index = 0

        self.visualize = True

        self.construct_probe_plan()

    def construct_probe_plan(self):
        # Iterate through each probe point
        for point in self.probe_points:
            # Extract the direction vector from the current probe point
            direction = point.direction
            added_to_existing_slice = False

            # Iterate through existing normal slices
            for existing_dir, points_list in self.normal_slices.items():
                # Calculate cosine similarity between the direction vectors
                similarity = calculate_cosine_similarity(direction, existing_dir)
                # Check if the similarity is above the tolerance threshold
                if similarity >= self.tolerance:
                    # Add the current probe point to the existing slice
                    points_list.append(point)
                    added_to_existing_slice = True
                    break

            # If the probe point was not added to any existing slice, create a new slice
            if not added_to_existing_slice:
                self.normal_slices[tuple(direction)] = [point]

        # Iterate through each direction and associated points list in normal_slices
        for direction, points_list in self.normal_slices.items():
            # Check if the number of points in the slice is below the minimum threshold
            if len(points_list) < self.min_points:
                closest_slice = None
                closest_similarity = 0

                # Iterate through existing slices to find the closest one based on cosine similarity
                for existing_dir, existing_points in self.normal_slices.items():
                    # Skip comparing a direction with itself
                    if existing_dir != direction:
                        # Calculate cosine similarity between the directions
                        similarity = calculate_cosine_similarity(direction, existing_dir)
                        # Update closest_slice if a higher similarity is found
                        if similarity > closest_similarity:
                            closest_similarity = similarity
                            closest_slice = existing_dir

                # If a closest slice is found, merge the points and remove the current slice
                if closest_slice is not None:
                    self.normal_slices[closest_slice].extend(points_list)
                    del self.normal_slices[direction]

        if self.visualize:
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

        # Sort slices based on proximity to the robot using a nearest neighbor algorithm
        point_list = list(self.normal_slices.keys())
        sorted_slices = nearest_neighbor_dict_sort(point_list, find_nearest_point_index(self.normal_slices,
                                                                                        pp.get_link_pose(
                                                                                            self.sim.sim_robot, 6)[0]))
        self.normal_slices = {point: self.normal_slices[point] for point in sorted_slices}

        # Convert normal_slices dictionary to a list of tuples for further processing
        self.normal_slices = [((x, y, z), data) for (x, y, z), data in self.normal_slices.items()]

    def get_next_confs(self):
        if self.side_index >= len(self.normal_slices):
            return None

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

            # todo add back off points.
            # new_point = pp.get_link_pose(self.sim.sim_robot, 6)[0]
            # new_point = np.add(new_point, [-.075, 0, 0])  # offset probe
            # self.sim.pos_probe_command = ProbePositionSetter(self.sim, new_point,
            #                                                  [0, 0, 0])
        else:
            self.probe_percentage = int(100 * (self.cur_point_index / (len(self.cur_path) - 1)))
            self.sim.controller.lbl_slice_index.setText(str(self.side_index))
            self.sim.controller.lbl_point_index.setText(str(self.cur_point_index))

            print("Processing slice i: ", self.side_index, " | Current point index: ",
                  self.cur_point_index)

            # get point from current index
            pt = self.cur_path[self.cur_point_index]

            # find angle between point and lineup vector
            goal_plat_rot = np.math.atan2(np.linalg.det([pt.direction[0:2], self.sim.lineup_normal[0:2]]),
                                  np.dot(pt.direction[0:2], self.sim.lineup_normal[0:2]))

            # rotate cur path
            temp = []
            for i in range(len(self.cur_path)):
                pos = np.dot(rotation_matrix_z(goal_plat_rot), self.cur_path[i].pos)
                dir = np.dot(rotation_matrix_z(goal_plat_rot), self.cur_path[i].direction)
                temp.append(AlignmentPoint(pos, dir))

            pos, dir = self.cur_path[self.cur_point_index].pos, [0, 0, 0]  # todo rbt orn?

            self.cur_path = temp
            self.cur_point_index += 1

            return (pos, dir), goal_plat_rot


class Charge:

    def __init__(self):
        self.start_time = 0
        self.end_time = 0

        self.is_charged = False


class Discharge:

    def __init__(self, stepper_inst, lds_inst, obj_z, cvr_len):
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
        self.cvr_len = cvr_len

    def discharge(self):
        self.stepper_board.home_scan()
        time.sleep(15)

        self.stepper_board.write_z((self.obj_z / 2) - 25, self.z_feed)
        self.stepper_board.write_x(102, self.x_feed)
        time.sleep(15)

        y = self.LDS.get_absolute_distance()
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
