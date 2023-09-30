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


def get_rotation_to_vector(static_vector, rotatable_vector):
    # normalize the vectors
    static_vector = static_vector / np.linalg.norm(static_vector)
    rotatable_vector = rotatable_vector / np.linalg.norm(rotatable_vector)
    # calculate the dot product of the two normalized vectors
    dot_product = np.dot(static_vector, rotatable_vector)
    # calculate the angle between the two vectors using the inverse cosine function
    angle = np.arccos(dot_product)
    return angle

def euclidean_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def nearest_neighbor(point, points):
    remaining_points = np.array(points)
    path = [point]

    while len(remaining_points) > 0:
        distances = np.linalg.norm(remaining_points - point, axis=1)
        nearest_idx = np.argmin(distances)
        nearest_point = remaining_points[nearest_idx]

        path.append(tuple(nearest_point))
        remaining_points = np.delete(remaining_points, nearest_idx, axis=0)
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