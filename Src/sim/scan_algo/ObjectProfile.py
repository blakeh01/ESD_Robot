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

from Src.robot.arm.RobotHandler import RobotHandler
from Src.sim.simulation import Simulation

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class ObjectProfile():

    def __init__(self, simulation: Simulation, flow, flow_args):
        self.sim = simulation
        self.probe_points = simulation.normal_point_cloud.alignment_points
        self.robot = simulation.robot_handler
        self.flow = flow

        # retrieve flow arguments
        self.rbt_max_speed = flow_args[0]
        self.rotator_feedrate = flow_args[1]
        self.grounding_interval = flow_args[2]
        self.measuring_time = flow_args[3]

        self.ground_flag = False # set to true when robot should ground after completed movement

        print("Generating object profile...")

    def initialize(self):
        pass

    def update(self):
        pass

class RotationallySymmetric(ObjectProfile):

    def __init__(self, simulation: Simulation, flow, flow_args):
        super(RotationallySymmetric, self).__init__(simulation, flow, flow_args)

        self.z_slices = {}
        self.tolerance = 0.005
        self.min_points_slice = 5
        self.visualize = True

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
                print("Detected a malformed slice! Try decreasing the tolerance of the object profile or decrease probing resolution!")
                # Find the neighboring slice with the most points
                neighboring_slices = [z for z in self.z_slices.keys() if abs(z - z_value) <= tolerance and z != z_value]
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
            #plt.legend(loc='upper right')

            # Show the plot
            plt.show()

        print("Generated " + str(len(self.z_slices.keys())) + " slices!")

class RectangularPrisms(ObjectProfile):

    def __init__(self, simulation: Simulation, flow, flow_args):
        super(RectangularPrisms, self).__init__(simulation, flow, flow_args)

        self.normal_slices = {}
        self.tolernace = 0.9
        self.min_points = 5

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
                if similarity >= tolerance:
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
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()

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