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
    Gets the force depending on the walls in the environment

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
  
  
def f_other_agents(agent_state: np.array, agent_positions: np.array, Di_length: float, Di_width: float) -> np.array:
    """
    Gets the force depending on the desired attractor

    :agent_state: Current agent state [x, y, v, phi] (NP Array)
    :agent_positions: All agent positions [[x0, y0, v0, phi0], [x1, y1, v1, phi1], ...] (NP Array)
    :Di_length: the length of the rectangle of influence
    :Di_width: the width of the rectangle of influence

    """
    # --- agent velocity ---
    v_agent = agent_state[2]
    phi_agent = agent_state[3]
    vx_agent = v_agent*np.cos(phi_agent)
    vy_agent = v_agent*np.sin(phi_agent)
    agent_dir = np.array([vx_agent, vy_agent])

    # --- basis vectors ---
    fwd = agent_dir / np.linalg.norm(agent_dir) # unit vector in forward direction
    side = np.array([-fwd[1], fwd[0]])          # vector orthogonal to the forward direction

    # --- vector from agent to all others ---
    vectors_to_others = agent_positions[:, :2] - agent_state[:2] 
    dist_to_others = np.linalg.norm(vectors_to_others, axis=1)

    # --- projections ---
    forward_dist = vectors_to_others @ fwd      # projection on forward direction 
    side_dist = vectors_to_others @ side        # projection on the side direction

    # --- filtering inside influence rectangle ---
    in_front     = forward_dist > 0              # only forward, not behind
    within_long  = forward_dist < Di_length      # within the length of rectangle 
    within_width = np.abs(side_dist) < (Di_width/2)  # within width of rectangle 
    not_too_close = dist_to_others > (Di_length - 1.5)  
    mask = in_front & within_long & within_width & not_too_close # masking-conditions
    idx = np.where(mask)[0]
    neighbours = agent_positions[mask]

    if len(neighbours) == 0:
        return np.array([0.0, 0.0])   # no force

    # --- neighbour velocities ---
    v_neighbours = neighbours[:, 2]
    phi_neighbours = neighbours[:, 3]
    vx_neighbours = v_neighbours*np.cos(phi_neighbours)
    vy_neighbours = v_neighbours*np.sin(phi_neighbours)
    neighbours_dir = np.stack([vx_neighbours, vy_neighbours], axis=1)

    # tangential forces
    vecs = vectors_to_others[idx]
    vecs_3d = np.hstack([vecs, np.zeros((len(vecs), 1))])
    agent_dir_3d  = np.array([agent_dir[0], agent_dir[1], 0.0])
    num = np.cross(np.cross(vecs_3d, agent_dir_3d), vecs_3d)   
    den = np.linalg.norm(num, axis=1, keepdims=True) + 1e-9  
    tangentials_3d = num / den
    tangentials = tangentials_3d[:, :2]  

    # --- weights ---
    d = dist_to_others[idx]
    w_d = (d - Di_length) ** 2
    dot_products = np.sum(neighbours_dir * agent_dir, axis=1)
    w_o = np.where(dot_products > 0, 1.2, 2.4)

    # --- forces ---
    forces = tangentials * w_d[:, None] * w_o[:, None]
    f_others = np.sum(forces, axis=0)

    return f_others

