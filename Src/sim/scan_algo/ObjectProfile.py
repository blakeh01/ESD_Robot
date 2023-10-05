'''

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
        - move probe back, rotate objetc
        - repeat

    Other objects ???
        - Would be cool to have an system for users to extend functionality with a script..?
'''
import math

import matplotlib.pyplot as plt
from Src.sim.Command import *
from Src.sim.simhelper import *
from Src.sim.simulation import Simulation
from Src.util.math_util import *


class ObjectProfile():

    def __init__(self, simulation: Simulation, flow, flow_args):
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
        pass

    def update(self):
        pass


class RotationallySymmetric(ObjectProfile):

    def __init__(self, simulation: Simulation, flow, flow_args):
        super(RotationallySymmetric, self).__init__(simulation, flow, flow_args)

        self.z_slices = {}
        self.z_sil_index = 0
        self.cur_slice = None
        self.tolerance = 0.005
        self.min_points_slice = 5
        self.visualize = True

        self.cur_path = None
        self.cur_point_index = 0

        self.next_groud_time = 0

        self.action_wait_start = 0
        self.action_wait_end = 0

        self.measure_flag = False

        self.initialize()

    def initialize(self):
        print("Grouping probe points by Z-axis with tolerance of ", self.tolerance, " m!")

        # group points by Z with a tolerance value
        for point in self.probe_points:
            z_value = round(point.pos[2], 3)  # Round to 2 decimal places for tolerance

            for existing_z, points_list in self.z_slices.items():
                if abs(existing_z - z_value) <= self.tolerance:
                    points_list.append(point)
                    break
            else:
                self.z_slices[z_value] = [point]

        # search for malformed slices, try to assign them to other slices.
        # if this happens, its relatively a 'bad' thing, so try to warn the user just in case.
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

        # visualize slices
        if self.visualize:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            # Create a list of colors for visualization
            colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

            # Scatter plot
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

            # Add legend
            # plt.legend(loc='upper right')

            # Show the plot
            plt.show()

        self.z_slices = sort_and_convert_to_list(self.z_slices)
        print("Generated " + str(len(self.z_slices)) + " slices!")

    '''
        Do I know what a state machine is? Yes.
        
        Do I realize that this would make this code about 10000x cleaner? Yes.
        
        But do I have the time to implement a full state machine for clean code? No. (lord forgive me)
    '''

    def update(self, time_elasped):
        if not self.can_run:
            return

        if self.cur_flow_idx + 1 > len(self.flow):
            print("Probe flow completed!")
            self.can_run = False
            return

        cur_flow = self.flow[self.cur_flow_idx]

        if isinstance(cur_flow, Wait):
            if cur_flow.end_time == 0 and cur_flow.start_time == 0:
                cur_flow.start_time = time_elasped
                cur_flow.end_time = cur_flow.start_time + cur_flow.wait_time

            if time_elasped >= cur_flow.end_time:
                print("Waiting completed... moving to next step!")
                self.cur_flow_idx += 1
        elif isinstance(cur_flow, Charge):
            if cur_flow.start_time == 0:
                cur_flow.start_time = time_elasped
                self.action_wait_end = time_elasped + 0.25
                self.sim.parent.btn_charge_done.setVisible(True)
                self.sim.parent.lbl_charge_warn.setVisible(True)
            else:
                if time_elasped >= self.action_wait_end:
                    self.action_wait_end = time_elasped + 0.25
                    self.sim.parent.lbl_charge_warn.setStyleSheet(
                        "background-color: lightgreen" if self.sim.parent.lbl_charge_warn.styleSheet() == "background-color: white" else "background-color: white")

            if self.charge_done_flag:
                cur_flow.end_time = time_elasped
                print("[USER] Charge Complete!")
                self.cur_flow_idx += 1
                self.charge_done_flag = False

        elif isinstance(cur_flow, Discharge):
            if cur_flow.start_time == 0:
                cur_flow.start_time = time_elasped
                input("DISCHARGE!")
                self.cur_flow_idx += 1
        elif isinstance(cur_flow, Probe):

            if cur_flow.start_time == 0:
                cur_flow.start_time = time_elasped
                self.z_sil_index = 0
                self.probe_percentage = 0
                print("PROBING! Sending robot home...")
            else:

                if self.z_sil_index >= len(self.z_slices):
                    print("Probing Completed!")
                    cur_flow.end_time = time_elasped
                    self.cur_flow_idx += 1
                    return

                if not self.cur_slice:
                    self.cur_slice = self.z_slices[self.z_sil_index]
                    print("Starting probe on Z-slice: ", self.z_sil_index, " Z-val: ", self.cur_slice[0])

                    print("Selecting point closest to robot!")
                    closest = find_closest_point(self.cur_slice[1], pp.get_link_pose(self.sim.sim_robot, 6)[0])

                    self.cur_path = find_alignment_point_path(closest, self.cur_slice[1])
                    self.sim.parent.plot_slice(self.cur_slice[1], self.cur_path)
                    print("Completed plotting slices! Beginning movements!")
                else:

                    if self.cur_point_index >= len(self.cur_path):
                        print("Reached end of slice!")
                        self.z_sil_index += 1
                        self.cur_point_index = 0
                        self.cur_slice = None
                        return

                    if self.sim.pos_plat_command.complete and self.sim.pos_probe_command.complete and not self.ground_flag and not self.measure_flag:
                        self.probe_percentage = int(100 * (self.cur_point_index / (len(self.cur_path) - 1)))
                        self.sim.parent.lbl_slice_index.setText(str(self.z_sil_index))
                        self.sim.parent.lbl_point_index.setText(str(self.cur_point_index))

                        if self.next_groud_time == 0:
                            self.next_groud_time = self.grounding_interval + time_elasped
                            print("Setting next ground time to: ", self.next_groud_time)

                        print("Processing slice i: ", self.z_sil_index, " | Current point index: ",
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

                        #orn = [0, math.atan2(self.cur_path[self.cur_point_index].direction[2], self.cur_path[self.cur_point_index].direction[1]) - np.pi, 0]
                        #print(orn)
                        self.cur_path = temp
                        self.sim.pos_probe_command = ProbePositionSetter(self.sim,
                                                                         self.cur_path[self.cur_point_index].pos,
                                                                         [0, 0, 0]
                                                                         )
                        self.measure_flag = True

                    if self.measure_flag and self.sim.pos_plat_command.complete and self.sim.pos_probe_command.complete and not self.ground_flag:
                        # i think i will quit coding for a long time after this. I am sick just looking at the
                        # repetitive code that ive written but have no choice as im enslaved and cannot think :]
                        if self.measuring_time > 0:
                            if self.action_wait_end == 0:
                                self.action_wait_end = time_elasped + self.measuring_time
                            elif time_elasped >= self.action_wait_end:
                                print("[FIX] BEEP PROBE VOLTAGE!")
                                self.cur_path[self.cur_point_index].measurement = 10
                                self.cur_point_index += 1
                                self.measure_flag = False
                                self.action_wait_end = 0
                        else:
                            print("[FIX] BEEP PROBE VOLTAGE!")
                            self.cur_path[self.cur_point_index].measurement = 10
                            self.cur_point_index += 1
                            self.measure_flag = False

        if self.next_groud_time <= time_elasped and self.next_groud_time != 0:
            print("Time to ground! Raising flag!")
            self.ground_flag = True
            self.next_groud_time = 0

        if self.ground_flag and self.sim.pos_probe_command.complete and self.sim.pos_plat_command.complete and self.action_wait_end == 0:
            new_point = pp.get_link_pose(self.sim.sim_robot, 6)[0]

            new_point = np.add(new_point, [-.15, 0, 0])  # offset probe
            self.sim.pos_probe_command = ProbePositionSetter(self.sim, new_point, [0, 0, 0]) # todo get joint orn?
            self.action_wait_end = time_elasped + 5  # wait 5 seconds to let system ground

        if self.action_wait_end != 0 and time_elasped >= self.action_wait_end and self.ground_flag:
            self.action_wait_end = 0
            self.ground_flag = False


class RectangularPrisms(ObjectProfile):

    def __init__(self, simulation: Simulation, flow, flow_args):
        super(RectangularPrisms, self).__init__(simulation, flow, flow_args)

        self.normal_slices = {}
        self.tolerance = 0.9
        self.min_points = 5

        self.cur_path = None
        self.cur_slice = None
        self.cur_point_index = 0
        self.side_index = 0

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

    def update(self, time_elasped):
        if not self.can_run:
            return

        if self.cur_flow_idx + 1 > len(self.flow):
            print("Probe flow completed!")
            self.can_run = False
            self.sim.parent.sim_stop()
            return

        cur_flow = self.flow[self.cur_flow_idx]

        if isinstance(cur_flow, Wait):
            if (cur_flow.end_time == 0 and cur_flow.start_time == 0):
                cur_flow.start_time = time_elasped
                cur_flow.end_time = cur_flow.start_time + cur_flow.wait_time

            if (cur_flow.end_time <= time_elasped):
                print("Waiting completed... moving to next step!")
                self.cur_flow_idx += 1
        elif isinstance(cur_flow, Charge):
            if cur_flow.start_time == 0:
                cur_flow.start_time = time_elasped
                self.action_wait_end = time_elasped + 0.25
                self.sim.parent.btn_charge_done.setVisible(True)
                self.sim.parent.lbl_charge_warn.setVisible(True)
            else:
                if time_elasped >= self.action_wait_end:
                    self.action_wait_end = time_elasped + 0.25
                    self.sim.parent.lbl_charge_warn.setStyleSheet(
                        "background-color: lightgreen" if self.sim.parent.lbl_charge_warn.styleSheet() == "background-color: white" else "background-color: white")

            if self.charge_done_flag:
                cur_flow.end_time = time_elasped
                print("[USER] Charge Complete!")
                self.cur_flow_idx += 1
                self.charge_done_flag = False
        elif isinstance(cur_flow, Discharge):
            if (cur_flow.start_time == 0):
                cur_flow.start_time = time_elasped
                input("DISCHARGE!")
                self.cur_flow_idx += 1
        elif isinstance(cur_flow, Probe):

            if (cur_flow.start_time == 0):
                cur_flow.start_time = time_elasped
                self.side_index = 0
                print("PROBING! Sending robot home...")
            else:

                if (self.side_index >= len(self.normal_slices) - 1):
                    print("Probing Completed!")
                    cur_flow.end_time = time_elasped
                    self.cur_flow_idx += 1
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
                    return

                if self.sim.pos_plat_command.complete and self.sim.pos_probe_command.complete and not self.ground_flag:

                    # if self.next_groud_time == 0:
                    #     self.next_groud_time = self.grounding_interval + time_elasped
                    #     print("Setting next ground time to: ", self.next_groud_time)

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
                                                                     goal_orn=[0, -np.pi/4, 0])

                    print("BEEP PROBE VOLTAGE!")
                    self.cur_path[self.cur_point_index].measurement = 10

                    self.cur_point_index += 1

        # if self.next_groud_time <= time_elasped and self.next_groud_time != 0:
        #     print("Time to ground! Raising flag!")
        #     self.ground_flag = True
        #     self.next_groud_time = 0
        #
        # if self.ground_flag and self.sim.pos_probe_command.complete and self.sim.pos_plat_command.complete:
        #     new_point = pp.get_link_pose(self.sim.sim_robot, 6)[0]
        #
        #     new_point = np.add(new_point, [-.1, 0, 0]) # offset probe
        #     self.sim.pos_probe_command = ProbePositionSetter(self.sim, new_point)
        #     self.ground_flag = False


class Charge():
    def __init__(self):
        self.start_time = 0
        self.end_time = 0


class Discharge():

    def __init__(self):
        self.start_time = 0
        self.end_time = 0


class Probe():

    def __init__(self):
        self.start_time = 0
        self.end_time = 0

        self.measured_points = []


class Wait():

    def __init__(self, wait_time):
        self.wait_time = wait_time

        self.start_time = 0
        self.end_time = 0


def sort_and_convert_to_list(z_slices):
    sorted_slices = sorted(z_slices.items(), key=lambda item: item[0], reverse=True)
    return sorted_slices
