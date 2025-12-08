import numpy as np
import geometry


def f_attractor(agent_state: np.array, attractor_position: np.array) -> np.array:
    """
    Gets the force depending on the desired attractor

    :agent_state: Current agent state [x, y, v, phi]
    :attractor_positions: Positions of all attractors [x_at, y_at] (NP Array)
    :returns: the force vector generated from the attractor point
    """
    f_at = attractor_position - agent_state[:2]

    return f_at


def f_walls(a, walls: np.array, box_width, box_length) -> np.array:
    """
    Gets the force depending on the walls in the environment

    :a: Current agent state [x, y, phi] (NP Array)
    :walls: Positions of all walls [[[x0, y0], [x1, y1]], ...] (NP Array)
    """

    walls_in_range = filter_walls(
        x=a.x,
        y=a.y,
        phi=a.angle,
        box_width=box_width,
        box_length=box_length,
        walls=walls,
    )

    if walls_in_range.size == 0:
        return np.array([0.0, 0.0])

    # Normals CCW to (x0, y0) -> (x1, y1)
    raw_normals = wall_normals(walls_in_range)
    normals = raw_normals.T 

    # Get adjusted normals and distances to agents
    # (On the same side of wall as agent)
    flat_walls = walls_in_range.flatten("C")
    x0 = flat_walls[0::4]
    y0 = flat_walls[1::4]
    wall_count = len(x0)

    force_sum = np.zeros(2)
    v_3d = np.array([a.v * np.cos(a.angle), a.v * np.sin(a.angle), 0.0])

    for i in range(wall_count):
        wall_start = np.array([x0[i], y0[i]])
        wall_start_to_agent = np.array([a.x, a.y]) - wall_start
        n = normals[i]
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-9:
            continue
        n = n / n_norm

        if np.dot(wall_start_to_agent, n) < 0:
            n = -n

        # Calculate actual forces
        n_3d = np.array([n[0], n[1], 0.0])

        # Equation (5) in Pelecjano, Allbeck & Badler
        c1 = np.cross(n_3d, v_3d)
        b = np.cross(c1, n_3d)

        if np.linalg.norm(b) < 1e-9:
            continue

        f_2d = b[:2] / np.linalg.norm(b)
    
        force_sum += f_2d

    return force_sum


def wall_normals(walls: np.array):
    """
    Gets the normal vector for a wall. If a wall is defined
    as points A and B, the vector will point 90 deg CCW from
    the vector from A to B
    :walls: Positions of all walls [[[x0, y0], [x1, y1]], ...] (NP Array)
    """

    # Prepare wall data format
    flat_walls = walls.flatten("C")
    x1 = flat_walls[0::4]
    y1 = flat_walls[1::4]
    x2 = flat_walls[2::4]
    y2 = flat_walls[3::4]

    rotation_matrix = np.array([[0, -1], [1, 0]])
    vectors = np.array([x2 - x1, y2 - y1])
    normals = rotation_matrix @ vectors
    return normals


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
    rect_line_1 = np.array([[0, box_width / 2], [box_length, box_width / 2]])
    rect_line_2 = np.array([[box_length, box_width / 2], [box_length, -box_width / 2]])
    rect_line_3 = np.array([[box_length, -box_width / 2], [0, -box_width / 2]])
    rect_line_4 = np.array([[0, -box_width / 2], [0, box_width / 2]])



    filtered_walls = []
    for i in range(np.size(x1_rotated)):
        x1 = x1_rotated[i]
        x2 = x2_rotated[i]
        y1 = y1_rotated[i]
        y2 = y2_rotated[i]
        wall_line = np.array([[x1, y1], [x2, y2]])

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


'''
def f_other_agents(
    agent_state: np.array, agent_positions: np.array, Di_length: float, Di_width: float
) -> np.array:
    """
    Gets the force depending on the other agents in the recttangle of influence

    :agent_state: Current agent state [x, y, v, phi] (NP Array)
    :agent_positions: All agent positions [[x0, y0, v0, phi0], [x1, y1, v1, phi1], ...] (NP Array)
    :Di_length: the length of the rectangle of influence
    :Di_width: the width of the rectangle of influence

    """
    if agent_positions.size == 0:
        return np.array([0.0, 0.0])
    
    # --- agent velocity ---
    v_agent = agent_state[2]
    phi_agent = agent_state[3]

    if v_agent < 1e-6:
        agent_dir = np.array([np.cos(phi_agent), np.sin(phi_agent)])
    else:
        agent_dir = np.array([v_agent*np.cos(phi_agent), v_agent*np.sin(phi_agent)])

    # --- basis vectors ---
    fwd = agent_dir / np.linalg.norm(agent_dir)  # unit vector in forward direction
    side = np.array([-fwd[1], fwd[0]])  # vector orthogonal to the forward direction

    # --- vector from agent to all others ---
    vectors_to_others = agent_positions[:, :2] - agent_state[:2]
    dist_to_others = np.linalg.norm(vectors_to_others, axis=1)

    # --- projections ---
    forward_dist = vectors_to_others @ fwd  # projection on forward direction
    side_dist = vectors_to_others @ side  # projection on the side direction

    # --- filtering inside influence rectangle ---
    in_front     = forward_dist > 0              # only forward, not behind
    within_long  = forward_dist < Di_length      # within the length of rectangle 
    within_width = np.abs(side_dist) < (Di_width/2)  # within width of rectangle 
    #not_too_close = dist_to_others > (Di_length - 1.5)  
    mask = in_front & within_long & within_width # masking-conditions
    idx = np.where(mask)[0]
    neighbours = agent_positions[mask]

    if len(neighbours) == 0:
        return np.array([0.0, 0.0])  # no force

    # --- neighbour velocities ---
    v_neighbours = neighbours[:, 2]
    phi_neighbours = neighbours[:, 3]
    vx_neighbours = v_neighbours * np.cos(phi_neighbours)
    vy_neighbours = v_neighbours * np.sin(phi_neighbours)
    neighbours_dir = np.stack([vx_neighbours, vy_neighbours], axis=1)

    # tangential forces
    vecs = vectors_to_others[idx]
    vecs_3d = np.hstack([vecs, np.zeros((len(vecs), 1))])
    agent_dir_3d = np.array([agent_dir[0], agent_dir[1], 0.0])
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
'''
def f_other_agents(
    agent_state: np.array, agent_positions: np.array, Di_length: float, Di_width: float
) -> np.array:
    """
    Gets the force depending on the other agents in the rectangle of influence.

    :agent_state: Current agent state [x, y, v, phi] (NP Array)
    :agent_positions: All agent states [[x0, y0, v0, phi0], ...] (NP Array)
    :Di_length: Length of rectangle of influence (forward)
    :Di_width:  Width of rectangle of influence (sideways)
    """
    if agent_positions.size == 0:
        return np.array([0.0, 0.0])
    
    # --- agent velocity / direction ---
    v_agent = agent_state[2]
    phi_agent = agent_state[3]

    if v_agent < 1e-6:
        agent_dir = np.array([np.cos(phi_agent), np.sin(phi_agent)])
    else:
        agent_dir = np.array([v_agent * np.cos(phi_agent), v_agent * np.sin(phi_agent)])

    # --- basis vectors ---
    fwd = agent_dir / (np.linalg.norm(agent_dir) + 1e-9)  # unit forward
    side = np.array([-fwd[1], fwd[0]])                    # unit "left"

    # --- vectors from agent to others ---
    vectors_to_others = agent_positions[:, :2] - agent_state[:2]
    dist_to_others = np.linalg.norm(vectors_to_others, axis=1)

    # --- projections ---
    forward_dist = vectors_to_others @ fwd   # along forward
    side_dist    = vectors_to_others @ side  # along left

    # --- filter inside influence rectangle ---
    in_front     = forward_dist > 0
    within_long  = forward_dist < Di_length
    within_width = np.abs(side_dist) < (Di_width / 2.0)
    mask = in_front & within_long & within_width

    if not np.any(mask):
        return np.array([0.0, 0.0])

    idx        = np.where(mask)[0]
    neighbours = agent_positions[mask]

    # --- neighbour velocities ---
    v_neighbours   = neighbours[:, 2]
    phi_neighbours = neighbours[:, 3]
    vx_neighbours  = v_neighbours * np.cos(phi_neighbours)
    vy_neighbours  = v_neighbours * np.sin(phi_neighbours)
    neighbours_dir = np.stack([vx_neighbours, vy_neighbours], axis=1)

    # --- relevant relative vectors ---
    vecs      = vectors_to_others[idx]          # (N, 2)
    d         = dist_to_others[idx]             # (N,)
    side_sel  = side_dist[idx]                  # (N,)

    # --- tangential directions (robust variant) ---
    tangentials = np.zeros_like(vecs)           # (N, 2)
    side_threshold = 0.05                       # "nästan rakt fram"-tröskel, tweaka vid behov

    agent_dir_3d = np.array([fwd[0], fwd[1], 0.0])

    for k in range(len(vecs)):
        rel = vecs[k]
        sd  = side_sel[k]

        if abs(sd) > side_threshold:
            # använd din korsprodukt–tangent som i HiDAC
            v3 = np.array([rel[0], rel[1], 0.0])
            num = np.cross(np.cross(v3, agent_dir_3d), v3)
            num2d = num[:2]
            nrm = np.linalg.norm(num2d)
            if nrm > 1e-9:
                tang = num2d / nrm     # normaliserad tangent
            else:
                # om den blir degenererad ändå → fallback
                tang = -side          # t.ex. alltid "sväng höger"
        else:
            # fallback: andra agenten nästan rakt fram → välj alltid samma sida
            # t.ex. alltid sväng åt höger (dvs motsatt "left"-vektor)
            tang = -side

        tangentials[k] = tang

    # --- weights ---
    w_d = (d - Di_length) ** 2
    dot_products = np.sum(neighbours_dir * fwd[None, :], axis=1)
    w_o = np.where(dot_products > 0, 1.2, 2.4)

    # --- forces ---
    forces = tangentials * w_d[:, None] * w_o[:, None]
    f_others = np.sum(forces, axis=0)

    return f_others


def f_repulsion_other_agents(agent_state: np.array, agent_radius: float, agent_epsilon: float, agent_positions: np.array,others_radius: np.array):
    """
    Computes repulsion force from nearby agents (overlap zone only)

    :agent_state: Current agent state [x, y, v, phi] (np Array)
    :agnet_radius: the radius of the agent (np array)
    :agent_epsilon: the personal space threshold for the agent
    :others_radius: the radius of all agents (np array) [r0, r1, r2, .....]
    :agent_positions: All agent positions [[x0, y0, v0, phi0], [x1, y1, v1, phi1], ...] (np Array)

    """
    if agent_positions.size == 0:
        return np.array([0.0, 0.0])
    
    # position of agent i
    pos_i = agent_state[:2]

    # vector from others to agent i
    vectors = pos_i - agent_positions[:, :2]               
    distances = np.linalg.norm(vectors, axis=1)           

    # repulsion threshold for each other agent
    repulsion_distances = agent_radius + others_radius + agent_epsilon

    # mask only overlapping agents
    mask = distances < repulsion_distances
    if not np.any(mask):
        return np.array([0.0, 0.0])
    
    # filter
    vectors = vectors[mask] # (pi-pj)
    distances = distances[mask] # dji
    repulsion_distances = repulsion_distances[mask] # ri + eps + rj

    # compute repulsion
    overlap = repulsion_distances - distances # ri + eps + rj - dji
    magnitudes = overlap / (distances + 1e-9)

    # final repulsion forces for each agent
    forces = vectors * (magnitudes[:, None])  

    # total repulsion 
    return np.sum(forces, axis=0)


def f_repulsion_walls(agent_state: np.array, agent_radius: float, agent_epsilon: float, box_width: float, box_length: float,walls: np.array) -> np.array:
    """
    computes the repulsion forces from the walls that are close 

    :x: Agent's x-position
    :y: Agent's y-position
    :box_width: Width of rectangle of influence
    :box_length: Length of rectangle of influence
    :walls: Positions of all walls [[[x0, y0], [x1, y1]], ...] (NP Array)
    """

    x, y, _, phi = agent_state
    pos = np.array([x, y])
    #fwd = np.array([np.cos(phi), np.sin(phi)])

    # filter walls to only close by
    candidate_walls = filter_walls(x, y, phi, box_width, box_length, walls)
    if candidate_walls.size == 0:
        return np.array([0.0, 0.0])


    F_total = np.array([0.0, 0.0])
    personal_radius = agent_radius + agent_epsilon

    # loop over relevant walls
    for wall in candidate_walls:
        A = wall[0]  # [x0, y0]
        B = wall[1]  # [x1, y1]
        AB = B - A

        # closest point on wall:
        AP = pos - A
        wall_squared_length = np.dot(AB, AB)
        proj = np.dot(AP, AB) /(wall_squared_length + 1e-12) # projection of position to wall
        proj_cut = np.clip(proj, 0.0, 1.0) # must be between 0= A and B = 1
        closest = A + proj_cut * AB      # closest point on wall
        vec = pos - closest              # the vector from the closest point on wall to the agent
        dist = np.linalg.norm(vec)       # distance used for the repulsion force
        #forward = np.dot(closest - pos, fwd)
        #if forward < 0:
        #  continue

        # only consider tgose within personal radius
        if dist >= personal_radius:
            continue  # no repulsion from wall

        # normalised normal
        n_wi = vec / (dist + 1e-9)

        # eq 11
        overlap = personal_radius - dist
        magnitude = overlap / (dist + 1e-9)
        F_R_wall = magnitude * n_wi

        F_total += F_R_wall

    return F_total


def f_repulsion(agent_state: np.array, agent_radius: float, agent_epsilon: float, agent_positions: np.array, others_radius: np.array, walls: np.array, box_width: float, box_length: float, lam_if_walls: float = 0.3, lam_default: float = 1.0 ) -> np.array:

    # repulsion from other agents
    F_agents = f_repulsion_other_agents(
        agent_state,
        agent_radius,
        agent_epsilon,
        agent_positions,
        others_radius
    )

    # repulsion from walls
    F_walls = f_repulsion_walls(
        agent_state,
        agent_radius,
        agent_epsilon,
        box_width,
        box_length,
        walls
    )

    # set lambda: decrease agent-repulsion if wall repulsion exists
    if np.linalg.norm(F_walls) > 0.0:
        lam = lam_if_walls   
    else:
        lam = lam_default    

    F_total = F_walls + lam * F_agents
    return F_total

