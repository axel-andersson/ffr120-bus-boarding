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


splits = 7
x = np.array([0, 1, 2, 3])
y = np.array([0, 0, 0, 0])

for _ in range(splits):
    angle = 2 * np.pi / splits
    x, y = rotate(x, y, angle)
    plt.scatter(x, y)

plt.xlim((-5, 5))
plt.ylim((-5, 5))
plt.grid()
plt.show()
