from agent_movement import f_attractor, f_walls, f_other_agents, f_repulsion, f_repulsion_other_agents
import numpy as np


class MovementAgent:


    # ─── Initialization ─────────────────────────────
    def __init__(self, x, y, angle, radius, epsilon, box_length, box_width, target: np.array, dt=0.1):
        # Force weights
        self.attractor_weight = 5
        self.wall_weight = 3
        self.other_agents_weight = 2

        # Position and movement
        self.x = x
        self.y = y
        self.angle = angle
        self.v = 0
        self.last_force = np.array([0, 0])
        self.a = 0.3  # constant acceleration
        self.dt = dt
        self.target = target

        # Movement modifiers
        self.inertia_factor = 0.2
        self.repulsion_scale = 0.3
        self.repulsion_stopping_threshold = 0.2
        self.stopping_probability = 0.5

        # Geometry and environment
        self.radius = radius
        self.epsilon = epsilon
        self.box_length = box_length
        self.box_width = box_width

        # Shaking / stopping control
        self.stopping_rule = False
        self.stop_timer = 0
        self.stress = False

        # Queuing behavior (influence disk )
        self.waiting_rule = False
        self.wait_timer = 0
        self.wait_disk_radius = 1.0
        self.wait_duration_range = (5, 15) 

        # Dynamic speed limits near goal
        self.v_far = 1.0     # Max speed far from target
        self.v_min = 0.2     # Minimum allowed v_max
        self.v_max = self.v_far
        self.brake_radius = 2.0  # Start slowing down within this distance


    # ─── Agent state as array ───────────────────────
    def state(self):
        return np.array([self.x, self.y, self.v, self.angle])


    # ─── Calculate movement and repulsion forces ────
    def movement_force(self, walls, agents: list["MovementAgent"]):
        state = self.state()

        other_states = np.array([ag.state() for ag in agents if ag is not self])
        others_radius = np.array([ag.radius for ag in agents if ag is not self])

        if other_states.size == 0:
            other_states = np.zeros((0, 4))
            others_radius = np.zeros((0,))

        attractor_force = f_attractor(state, self.target)
        walls_force = f_walls(self, walls, self.box_length, self.box_width)
        other_agents_force = f_other_agents(state, other_states, self.box_length, self.box_width)

        F_to_vec = (
            self.inertia_factor * self.last_force
            + self.attractor_weight * attractor_force
            + self.wall_weight * walls_force
            + self.other_agents_weight * other_agents_force
        )

        repulsion = f_repulsion(
            state,
            self.radius,
            self.epsilon,
            other_states,
            others_radius,
            walls,
            self.box_width,
            self.box_length,
        )

        f_rep_agents = f_repulsion_other_agents(
            state,
            self.radius,
            self.epsilon,
            other_states,
            others_radius,
        )

        return F_to_vec, repulsion, f_rep_agents


    # ─── Handle repulsion-based stopping rule ───────
    def _update_stopping_rule(self, f_rep_agents: np.array):
        v_vec = np.array([np.cos(self.angle), np.sin(self.angle)]) * self.v
        if np.linalg.norm(v_vec) < 1e-6:
            return

        rep_norm = np.linalg.norm(f_rep_agents)
        if (
            rep_norm > self.repulsion_stopping_threshold
            and np.dot(f_rep_agents, v_vec) < 0
            and not self.stress
            and not self.stopping_rule
            and np.random.rand() < self.stopping_probability
        ):
            self.stopping_rule = True
            self.stop_timer = np.random.randint(5, 20)

        if self.stopping_rule:
            self.stop_timer -= 1
            if self.stop_timer <= 0:
                self.stopping_rule = False

    # ─── Handle waiting rule based on influence disk ───────
    def check_waiting_zone(self, agents: list["MovementAgent"]):

        my_pos = np.array([self.x, self.y])
        fwd = np.array([np.cos(self.angle), np.sin(self.angle)])

        for other in agents:
            if other is self:
                continue
            
            other_pos = np.array([other.x, other.y])
            vec = other_pos - my_pos
            dist = np.linalg.norm(vec)
            if dist > self.wait_disk_radius:
                continue
            # check if other is roughly in front (dot product > 0.5)
            if np.dot(vec, fwd) / (np.linalg.norm(vec) + 1e-9) > 0.9:
                return True
        
        return False


    # ─── Main update per timestep ───────────────────
    def update(self, walls: np.array, agents: list["MovementAgent"]):
        pos = np.array([self.x, self.y])
        to_target = self.target - pos
        dist = np.linalg.norm(to_target)

        # Adjust v_max based on braking distance
        if dist < self.brake_radius:
            factor = dist / self.brake_radius
            self.v_max = max(self.v_min, self.v_far * factor)
        else:
            self.v_max = self.v_far

        # Stop if at goal
        stop_radius = self.radius + 0.1
        if dist < stop_radius:
            self.v = 0.0
            self.last_force = np.array([0.0, 0.0])
            return
        
        # Queuing behavior: influence disk
        if not self.waiting_rule and self.check_waiting_zone(agents):
            self.waiting_rule = True
            self.wait_timer = np.random.randint(*self.wait_duration_range)

        if self.waiting_rule:
            self.wait_timer -= 1
            if self.wait_timer <= 0:
                self.waiting_rule = False

        # Compute movement forces
        F_to_vec, repulsion, f_rep_agents = self.movement_force(walls, agents)
        self._update_stopping_rule(f_rep_agents)

        alpha = 0.0 if (self.stopping_rule or self.waiting_rule) else 1.0

        if np.linalg.norm(F_to_vec) > 1e-6:
            f_to = F_to_vec / np.linalg.norm(F_to_vec)
        else:
            f_to = np.array([np.cos(self.angle), np.sin(self.angle)])

        # Accelerate toward v_max
        self.v = min(self.v + self.a * self.dt, self.v_max)

        # Movement vector
        delta_desired = alpha * self.v * self.dt * f_to
        delta_rep = self.repulsion_scale * repulsion * self.dt

        move_vec = delta_desired + delta_rep
        self.x += move_vec[0]
        self.y += move_vec[1]

        # Update heading
        if np.linalg.norm(move_vec) > 1e-6:
            self.angle = np.arctan2(move_vec[1], move_vec[0])

        # Store force for inertia
        self.last_force = F_to_vec




