import numpy as np
import matplotlib.pyplot as plt


def line_intersection(line_1, line_2):
    """
    Finds intersection point between two lines.
    Returns [x, y] if exists and None if it doesn't cross.
    Colinear lines are seen as non-intersecting for simplicity.

    :line_a: Line defined as [[x0, y0], [x1, y1]] (numpy array)
    :line_b: Line defined as [[x0, y0], [x1, y1]] (numpy array)
    """

    x1 = line_1[0, 0]
    x2 = line_1[1, 0]
    x3 = line_2[0, 0]
    x4 = line_2[1, 0]

    y1 = line_1[0, 1]
    y2 = line_1[1, 1]
    y3 = line_2[0, 1]
    y4 = line_2[1, 1]

    t_numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
    t_denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    u_numerator = (x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)
    u_denomenator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    # Handle case where lines are colinear
    if (
        t_numerator == 0
        and t_denominator == 0
        and u_numerator == 0
        and u_denomenator == 0
    ):
        return None
    # Handle case where lines are parallell
    elif t_denominator == 0 or u_denomenator == 0:
        return None

    # Handle standard case
    t = t_numerator / t_denominator
    u = -u_numerator / u_denomenator

    if t >= 0 and t <= 1 and u >= 0 and u <= 1:
        p1 = line_1[0]
        p2 = line_1[1]
        intersect_position = p1 + t * (p2 - p1)
        return intersect_position
    else:
        return None


def point_line_intersects(point, line):
    x_A = line[0][0]
    y_A = line[0][1]
    x_B = line[1][0]
    y_B = line[1][1]

    x_C = point[0]
    y_C = point[1]

    # Point is on extended line ONLY if cross product is 0
    if np.cross(np.array([x_A, y_A, 0]), np.array([x_C, y_C, 0])) != 0:
        return False

    # Point is on line if it can be paremetrized
    delta_x = x_B - x_A
    delta_y = y_B - y_A

    if delta_x != 0:
        t = (x_C - x_A) / delta_x
        return 0 <= t and t <= 1
    elif delta_y != 0:
        t = (y_C - y_A) / delta_x
        return 0 <= t and t <= 1
    else:
        raise "Line segment has no length."


def translate(x, y, dx, dy):
    """
    Translates positions based on deltas

    :x: array of x-values
    :y: array of y-values
    :dx: x-delta
    :dy: y-delta
    """
    new_x = x + dx
    new_y = y + dy
    return new_x, new_y


def rotate(x, y, theta):
    """
    Rotate positions counter-clockwise around origin

    :x: array of x-values
    :y: array of y-values
    :theta: rotation in radians (positive if ccw)
    """
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    positions = np.array([x, y])
    new_positions = np.matmul(R, positions)
    new_x = new_positions[0]
    new_y = new_positions[1]
    return new_x, new_y
