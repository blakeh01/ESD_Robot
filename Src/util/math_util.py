import numpy as np
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


def find_corner(points):
    highest_point = None
    weight = float('-inf')

    for point in points:
        x, y, z = point.pos
        if abs(x) + abs(y) + abs(z) >= weight:
            highest_point = point
            weight = abs(x) + abs(y) + abs(z)

    return highest_point


def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1.pos) - np.array(point2.pos))


# two different functions, one for alignment points, one for regular pos.... this is foul im sorry
def distance(point1, point2):
    return np.linalg.norm(np.array(point1)[:2] - np.array(point2)[:2])

def weighted_euclidean_distance(point1, point2, z_weight=1.0):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2 + ((z1 - z2) * z_weight) ** 2) ** 0.5
   # z_difference = abs(z1 - z2)
    #weighted_distance = distance + z_weight * z_difference
    return distance

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


def nearest_neighbor_dict_sort(points, start_index=0):
    sorted_points = [points[start_index]]
    unvisited_points = list(points[:start_index] + points[start_index + 1:])

    while unvisited_points:
        current_point = sorted_points[-1]
        nearest_point = min(unvisited_points, key=lambda p: weighted_euclidean_distance(current_point, p, z_weight=2))
        sorted_points.append(nearest_point)
        unvisited_points.remove(nearest_point)

    return sorted_points


def find_nearest_point_index(dictionary, target_point):
    min_distance = float('inf')
    nearest_point_index = None

    for i, (point, _) in enumerate(dictionary.items()):
        d = distance(point, target_point)
        if d < min_distance:
            min_distance = d
            nearest_point_index = i

    return nearest_point_index


def find_closest_point(slice_points, target_point):
    closest_point = None
    closest_distance = float('inf')

    for point in slice_points:
        distance = np.linalg.norm(np.array(point.pos) - np.array(target_point))
        if distance < closest_distance:
            closest_distance = distance
            closest_point = point

    return closest_point
