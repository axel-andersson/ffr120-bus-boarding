from agent_movement import f_attractor, f_walls, f_other_agents, f_repulsion
import numpy as np


class MovementAgent:
    def __init__(self, x, y, angle, radius, epsilon, box_length, box_width):
        # WEIGHTS
        self.attractor_weight = 1
        self.wall_weight = 1
        self.other_agents_weight = 1

        # POSITION & MOVEMENT
        self.x = x
        self.y = y
        self.angle = angle
        self.v = 0
        self.last_force = np.array([0, 0])  # Handles inertia

        # CONSTANTS
        self.v_max = 1  # Individual maximum velocity

        # REPULSION / GEOMETRY
        self.radius = radius
        self.epsilon = epsilon
        self.box_length = box_length
        self.box_width = box_width

    def state(self):
        return np.array([self.x, self.y, self.v, self.angle])

    def movement_force(self, attractor_position, walls, agents: list["MovementAgent"]):

        state = self.state()
        other_states = np.array([ag.state() for ag in agents if ag is not self])
        others_radius = np.array([ag.radius for ag in agents if ag is not self])

        attractor_force = f_attractor(state, attractor_position)
        walls_force = f_walls(state, walls)
        other_agents_force = f_other_agents(state, other_states, self.box_length, self.box_width)
        Repulsion = f_repulsion(state, self.radius, self.epsilon, other_states, others_radius, walls, self.box_length, self.box_width)

        total_force = (
            self.last_force
            + self.attractor_weight * attractor_force
            + self.wall_weight * walls_force
            + self.other_agents_weight * other_agents_force
        )
        return total_force
