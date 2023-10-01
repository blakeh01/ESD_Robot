import numpy as np
import math
from scipy.spatial import cKDTree

RAD_TO_DEG = 57.2957795
DEG_TO_RAD = 1.0 / RAD_TO_DEG


# Chunks out an array into many sub arrays depending on the size limit.
def chunk(array, size_limit):
    out = []
    l = len(array)
    s = 0

    while l - s > 0:
        out.append(array[s:s + size_limit])
        s += size_limit

    return out


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def rotation_matrix_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


def unify_points(points, tolerance):
    tree = cKDTree(points)
    new_points = []
    for point in points:
        _, indices = tree.query(point, k=2)
        if indices[0] == indices[1]:
            continue
        point1 = points[indices[0]]
        point2 = points[indices[1]]
        distance = ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2) ** 0.5
        if distance > tolerance:
            new_points.append(point1)
        else:
            new_point = [(point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2, (point1[2] + point2[2]) / 2]
            new_points.append(new_point)
    return new_points

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1.pos) - np.array(point2.pos))

def find_alignment_point_path(point, alignment_points):
    remaining_points = alignment_points.copy()
    path = [point]

    while remaining_points:
        distances = [euclidean_distance(point, candidate_point) for candidate_point in remaining_points]
        nearest_idx = np.argmin(distances)
        nearest_point = remaining_points[nearest_idx]

        path.append(nearest_point)
        remaining_points.remove(nearest_point)
        point = nearest_point

    return path

def find_closest_point(slice_points, target_point):
    closest_point = None
    closest_distance = float('inf')

    for point in slice_points:
        distance = np.linalg.norm(np.array(point.pos) - np.array(target_point))
        if distance < closest_distance:
            closest_distance = distance
            closest_point = point

    return closest_point