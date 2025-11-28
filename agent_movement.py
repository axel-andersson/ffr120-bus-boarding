import numpy as np
import geometry


def f_attractor(agent_state: np.array, attractor_position: np.array) -> np.array:
    """
    Gets the force depending on the desired attractor

    :agent_state: Current agent state [x, y, phi] (NP Array)
    :attractor_positions: Positions of all attractors [x_at, y_at] (NP Array)
    :returns: the force vector generated from the attractor point
    """
    f_at = attractor_position - agent_state[:2]
    
    return f_at


def f_walls(agent_state: np.array, walls: np.array) -> np.array:
    """
    Gets the force depending on the desired attractor

    :agent_state: Current agent state [x, y, phi] (NP Array)
    :walls: Positions of all walls [[[x0, y0], [x1, y1]], ...] (NP Array)
    """
    pass


def filter_walls(x, y, phi, box_width, box_length, walls):
    """
    Filters walls to only include those inside the rectangle of influence for an agent.
    Box has one side with length "box_width" centered in (x, y). It has its opposite side
    offset box_length in the direction of phi

    :x: Agent's x-position
    :y: Agent's y-position
    :box_width: Width of rectangle of influence
    :box_length: Length of rectangle of influence
    :walls: Positions of all walls [[[x0, y0], [x1, y1]], ...] (NP Array)
    """

    # Prepare wall data format for transform
    flat_walls = walls.flatten("C")
    x1_normal = flat_walls[0::4]
    y1_normal = flat_walls[1::4]
    x2_normal = flat_walls[2::4]
    y2_normal = flat_walls[3::4]

    # Transform system to have origin at (x, y)
    # and (1, 0) be in the direction the agent is facing.
    x1_translated, y1_translated = geometry.translate(x1_normal, y1_normal, -x, -y)
    x2_translated, y2_translated = geometry.translate(x2_normal, y2_normal, -x, -y)

    x1_rotated, y1_rotated = geometry.rotate(x1_translated, y1_translated, -phi)
    x2_rotated, y2_rotated = geometry.rotate(x2_translated, y2_translated, -phi)

    # Check if inside rectangle of influence, in transformed coordinates
    rect_line_1 = np.array([0, box_width / 2], [box_length, box_width / 2])
    rect_line_2 = np.array([box_length, box_width / 2], [box_length, -box_width / 2])
    rect_line_3 = np.array([box_length, -box_width / 2], [0, -box_width / 2])
    rect_line_4 = np.array([0, -box_width / 2], [0, box_width / 2])

    filtered_walls = []
    for i in range(np.size(x1_rotated)):
        x1 = x1_rotated[i]
        x2 = x2_rotated[i]
        y1 = y1_rotated[i]
        y2 = y2_rotated[i]
        wall_line = np.array[[x1, y1], [x2, y2]]

        # Are any of the points inside the range?
        if (
            x1 >= 0,
            x1 <= box_length,
            y1 >= -box_width / 2,
            y1 <= box_width / 2,
        ):
            filtered_walls.append(walls[i])
        elif (
            x2 >= 0,
            x2 <= box_length,
            y2 >= -box_width / 2,
            y2 <= box_width / 2,
        ):
            filtered_walls.append(walls[i])

        # Does the wall intersect any of the edges?
        elif geometry.line_intersection(wall_line, rect_line_1) is not None:
            filtered_walls.append(walls[i])
        elif geometry.line_intersection(wall_line, rect_line_2) is not None:
            filtered_walls.append(walls[i])
        elif geometry.line_intersection(wall_line, rect_line_3) is not None:
            filtered_walls.append(walls[i])
        elif geometry.line_intersection(wall_line, rect_line_4) is not None:
            filtered_walls.append(walls[i])

    return np.array(filtered_walls)


def f_other_agents(agent_state: np.array, agent_positions: np.array) -> np.array:
    """
    Gets the force depending on the desired attractor

    :agent_state: Current agent state [x, y, phi] (NP Array)
    :agent_positions: All agent positions [[x0, y0, phi0], [x1, y1, phi1], ...] (NP Array)
    """
    pass
