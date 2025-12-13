from agent_movement import (
    f_attractor,
    f_walls,
    f_other_agents,
    f_repulsion,
    f_repulsion_other_agents,
)
import numpy as np


class MovementAgent:

    # ─── Initialization ─────────────────────────────
    def __init__(self, x, y, angle, radius, epsilon, box_length, box_width, dt=0.1):
        # Force weights
        self.attractor_weight = 5
        self.wall_weight = 0.1
        self.other_agents_weight = 2

        # Position and movement
        self.x = x
        self.y = y
        self.angle = angle
        self.v = 0
        self.last_force = np.array([0, 0])
        self.a = 0.3  # constant acceleration
        self.dt = dt

        # Vehicle specific navigation
        self.is_sitting = False
        self.mounting_point = None
        self.is_entering = False
        self.is_exiting = False

        self.pre_hold_target_queue = []
        self.post_hold_target_queue = []

        # Targets
        self.target_queue = []
        self.target = [self.x, self.y]
        self.reached_final_target = False

        # Ticket control check
        self.ticket_check_pos = np.array(
            [3, 2.5]
        )  # <-------- one point where ticket check is being made
        self.ticket_checked = False
        self.checking_ticket = False
        self.check_ticket_duration_range = (0.5, 3)  # in seconds
        self.ticket_check_radius = 1  # <------ how far from ticket check point you must be to stop and check your ticket

        # Movement modifiers
        self.inertia_factor = 0.1
        self.repulsion_scale = 0.05
        self.repulsion_stopping_threshold = 0.1
        self.stopping_probability = 0.2

        # Geometry and environment
        self.radius = radius
        self.epsilon = epsilon
        self.box_length = box_length
        self.box_width = box_width

        # Shaking / stopping control (repulsion based)
        self.stopping_duration_range = (0.5, 3.0)  # in seconds
        self.stopping_rule = False
        self.stop_timer = 0
        self.stress = False

        # Queuing behavior (influence disk )
        self.waiting_rule = False
        self.wait_timer = 0
        self.wait_disk_radius = 0.5
        self.wait_duration_range = (0.2, 0.7)  # in seconds

        # Dynamic speed limits near goal
        self.v_far = 1  # Max speed far from target
        self.v_min = 0.2  # Minimum allowed v_max
        self.v_max = self.v_far
        self.brake_radius = 1.0  # Start slowing down within this distance

    # ─── Agent state as array ───────────────────────
    def state(self):
        return np.array([self.x, self.y, self.v, self.angle])

    # ─── Change attraction poiint if prevoius is reached ────
    def set_target(self):

        n = len(self.target_queue)

        if n == 0:
            return self.target, self.target_queue

        self.target = self.target_queue[0]
        self.target_queue = self.target_queue[1:]
        return self.target, self.target_queue

    # ─── Stop if  position is at ticket control position ────
    def check_ticket(self):

        pos = np.array([self.x, self.y])
        to_ticket_control = self.ticket_check_pos - pos
        dist = np.linalg.norm(to_ticket_control)

        if dist < self.ticket_check_radius:
            return True

    # ─── Calculate movement and repulsion forces ────
    def movement_force(self, walls, agents: list["MovementAgent"]):
        state = self.state()

        other_states = np.array([ag.state() for ag in agents if ag is not self])
        others_radius = np.array([ag.radius for ag in agents if ag is not self])

        if other_states.size == 0:
            other_states = np.zeros((0, 4))
            others_radius = np.zeros((0,))

        attractor_force = f_attractor(state, self.target)
        walls_force = f_walls(self, walls, self.box_width, self.box_length)
        other_agents_force = f_other_agents(
            state, other_states, self.box_length, self.box_width
        )

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

        WALKING_PRIORITY = 10
        f_rep_agents_modified = (
            f_rep_agents / WALKING_PRIORITY
            if self.is_entering or self.is_exiting
            else f_rep_agents
        )

        return F_to_vec, repulsion, f_rep_agents_modified

    # ─── Handle repulsion-based stopping rule ───────
    def _update_stopping_rule(self, f_rep_agents: np.array):
        v_vec = np.array([np.cos(self.angle), np.sin(self.angle)]) * self.v
        rep_norm = np.linalg.norm(f_rep_agents)

        # If stop rule already triggered, calculate timer and return
        if self.stopping_rule:
            self.stop_timer -= 1
            if self.stop_timer <= 0:
                self.stopping_rule = False
            return

        # If no movement has been created yet, return
        if np.linalg.norm(v_vec) < 1e-6:
            return

        # Otherwise trigger stop
        if (
            rep_norm > self.repulsion_stopping_threshold
            and np.dot(f_rep_agents, v_vec) < 0
            and not self.stress
            and np.random.rand() < self.stopping_probability
        ):
            self.stopping_rule = True
            duration_s = np.random.uniform(*self.stopping_duration_range)  # to seconds
            self.stop_timer = max(1, int(duration_s / self.dt))

    # ─── Handle waiting rule based on influence disk ───────
    def check_waiting_zone(self, agents: list["MovementAgent"]):

        if self.is_entering:
            waitable_agents = list(filter(lambda a: a.is_entering, agents))
        elif self.is_exiting:
            waitable_agents = list(filter(lambda a: a.is_exiting, agents))
        else:
            waitable_agents = []

        my_pos = np.array([self.x, self.y])
        fwd = np.array([np.cos(self.angle), np.sin(self.angle)])

        for other in waitable_agents:
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

        # Do not move people sitting down
        if self.is_sitting:
            return

        pos = np.array([self.x, self.y])
        to_target = self.target - pos
        dist = np.linalg.norm(to_target)

        if dist < 0.6:
            self.target, self.target_queue = self.set_target()
            to_target = self.target - pos
            dist = np.linalg.norm(to_target)

        is_final_target = len(self.target_queue) == 0

        # Adjust v_max based on braking distance, if current target is the final attraction point
        if dist < self.brake_radius and is_final_target:
            factor = dist / self.brake_radius
            self.v_max = max(self.v_min, self.v_far * factor)
        else:
            self.v_max = self.v_far

        # Stop if ticket check is required
        if not self.ticket_checked and self.check_ticket():
            self.ticket_checked = True
            self.checking_ticket = True
            duration_s = np.random.uniform(
                *self.check_ticket_duration_range
            )  # convert to seconds
            self.check_ticket_timer = max(1, int(duration_s / self.dt))

        if self.checking_ticket:
            self.check_ticket_timer -= 1
            if self.check_ticket_timer <= 0:
                self.checking_ticket = False

        # Stop if at goal (last attractor) and also set
        # to move
        stop_radius = self.radius + 0.1
        if dist < stop_radius and is_final_target:
            self.v = 0.0
            self.last_force = np.array([0.0, 0.0])
            self.reached_final_target = True

        # Queuing behavior: influence disk
        if not self.waiting_rule and self.check_waiting_zone(agents):
            self.waiting_rule = True
            duration_s = np.random.uniform(*self.wait_duration_range)
            self.wait_timer = max(1, int(duration_s / self.dt))  # seconds

        if self.waiting_rule:
            self.wait_timer -= 1
            if self.wait_timer <= 0:
                self.waiting_rule = False

        # Compute movement forces
        F_to_vec, repulsion, f_rep_agents = self.movement_force(walls, agents)
        self._update_stopping_rule(f_rep_agents)

        # bug check
        reason = []
        if self.stopping_rule:
            reason.append("STOPPING_RULE")
        if self.waiting_rule:
            reason.append("WAITING_RULE")
        if self.checking_ticket:
            reason.append("TICKET")

        # precis innan du beräknar alpha:
        if reason:
            pass
            # print(f"Agent debug: x={self.x:.2f}, y={self.y:.2f}, orsaker: {reason}")

        # Move as long as not stop because of various reasons
        alpha = (
            0.0
            if (self.stopping_rule or self.waiting_rule or self.checking_ticket)
            else 1.0
        )

        # normalize forces
        if np.linalg.norm(F_to_vec) > 1e-6:
            f_to = F_to_vec / np.linalg.norm(F_to_vec)
        else:
            f_to = np.array([np.cos(self.angle), np.sin(self.angle)])

        # Accelerate toward v_max
        if self.stopping_rule or self.waiting_rule or self.checking_ticket:
            self.v = 0.0
        else:
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
