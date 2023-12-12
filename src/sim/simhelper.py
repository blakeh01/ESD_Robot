import pybullet as p
import pybullet_planning as pp

from src.util.math_util import *


def get_normal_point_cloud(pts_per_slice, z_limits, z_density, probe_dist, obj_center, draw_ray_start=False,
                           draw_cloud=True):
    ray_start = []
    ray_end = []

    z_space = np.linspace(start=z_limits[0], stop=z_limits[1], num=z_density, endpoint=True)
    phi_space = np.linspace(start=0, stop=2 * np.pi, num=pts_per_slice, endpoint=False)  # no endpoint as 0 = 2pi
    rho = .200  # 200 mm

    for phi in phi_space:
        for z in z_space:
            x, y, z = (
                rho * np.cos(phi) + obj_center[0],
                rho * np.sin(phi) + obj_center[1],
                z
            )

            ray_start.append([x, y, z])
            ray_end.append([obj_center[0], obj_center[1], z])

    if draw_ray_start: p.addUserDebugPoints(ray_start, [[255, 0, 0]] * len(ray_start), 5)

    if len(ray_start) > p.MAX_RAY_INTERSECTION_BATCH_SIZE:
        print("[SIM] Attempted raycast beyond bounded limit!")
        return

    ray_batch = p.rayTestBatch(ray_start, ray_end, numThreads=0)
    alignment_points = []

    for result in ray_batch:
        if result[1] != -1:
            hit_pos = result[3]
            hit_norm = result[4]

            offset_pos = np.add(hit_pos, np.dot(hit_norm, probe_dist))
            alignment_points.append(AlignmentPoint(offset_pos, hit_norm))

    point_cloud = PointCloud(alignment_points)

    if draw_cloud:
        point_cloud.draw_point_cloud()

    return point_cloud


class AlignmentPoint:
    """
        Stores a point, a normal, and a isMeasured field. Will be used when probing the robot.

    """

    def __init__(self, pos, direction):
        self.pos = pos
        self.direction = direction
        self.measurement = 0


class PointCloud:
    """
        Stores a list of alignment points and draws them with platform rotation.
    """

    def __init__(self, alignment_points):
        self.alignment_points = alignment_points
        self.disp_pc_points = None  # used in drawing points

    def get_num_points(self):
        return len(self.alignment_points)

    def draw_point_cloud(self, color=[0, 0, 255], size=5, rotation=0):
        pts = []
        rot_matrix = rotation_matrix_z(rotation)

        for ap in self.alignment_points:
            pts.append(rot_matrix @ ap.pos)
        if self.disp_pc_points is None:
            self.disp_pc_points = p.addUserDebugPoints(pts, [color] * len(pts), size)
        else:
            p.addUserDebugPoints(pts, [color] * len(pts), size, replaceItemUniqueId=self.disp_pc_points)

    def delete_cloud(self):
        p.removeUserDebugItem(self.disp_pc_points)


def draw_axis(pose, refs, length=0.1):
    origin_world = pp.tform_point(pose, np.zeros(3))
    handles = []
    for k in range(3):
        axis = np.zeros(3)
        axis[k] = 1
        axis_world = pp.tform_point(pose, length * axis)
        handles.append(p.addUserDebugLine(origin_world, axis_world, lineColorRGB=axis, replaceItemUniqueId=refs[k]))

    return handles


def draw_tip_axis(rbt_id, refs, tip_id=6):
    point, quat = pp.get_link_pose(rbt_id, tip_id)

    draw_axis((point, quat), refs)


def draw_world_axis(size, lineWidth):
    rotation_matrix = np.array(p.getMatrixFromQuaternion([0, 0, 0, 1])).reshape(3, 3)
    position = np.array([0, 0, 0])
    x = rotation_matrix.dot([size, 0, 0])
    y = rotation_matrix.dot([0, size, 0])
    z = rotation_matrix.dot([0, 0, size])

    # add axis X (red), Y (green), and Z (blue)
    p.addUserDebugLine(position, position + x, [1, 0, 0], lineWidth=lineWidth)
    p.addUserDebugLine(position, position + y, [0, 1, 0], lineWidth=lineWidth)
    p.addUserDebugLine(position, position + z, [0, 0, 1], lineWidth=lineWidth)


def focus_camera(focus_point, offset):
    camera_pt = np.array(focus_point) + np.array(offset)
    pp.set_camera_pose(tuple(camera_pt), focus_point)
