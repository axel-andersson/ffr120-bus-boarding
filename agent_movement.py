import numpy as np


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
    :walls: Positions of all wall ends [[xA0, yA0, xB0, yB0], ...] (NP Array)
    """
    pass


def f_other_agents(agent_state: np.array, agent_positions: np.array) -> np.array:
    """
    Gets the force depending on the desired attractor

    :agent_state: Current agent state [x, y, phi] (NP Array)
    :agent_positions: All agent positions [[x0, y0, phi0], [x1, y1, phi1], ...] (NP Array)
    """
    pass
