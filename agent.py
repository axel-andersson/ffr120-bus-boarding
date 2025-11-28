from agent_movement import f_attractor, f_walls, f_other_agents
import numpy as np


class MovementAgent:
    def __init__(self, x, y, angle):
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

    def state(self):
        return np.array([self.x, self.y, self.angle])

    def movement_force(self, attractor_position, walls, other_agents_positions):
        state = self.state()
        attractor_force = f_attractor(state, attractor_position)
        walls_force = f_walls(state, walls)
        other_agents_force = f_other_agents(state, other_agents_positions)

        total_force = (
            self.last_force
            + self.attractor_weight * attractor_force
            + self.wall_weight * walls_force
            + self.other_agents_weight * other_agents_force
        )
        return total_force
