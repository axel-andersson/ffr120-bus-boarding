from vehicle_definitions import articulated_bus
from vehicle import SimSpace
from agent import MovementAgent
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np


def run_simulation(vehicle, current_count, entering_count, exiting_count):
    pass


def init_current_passengers(vehicle: SimSpace, count):
    agents = []
    standing_positions = []
    for _ in range(count):
        seat, target_pos = vehicle.find_boarding_target()

        agent_pos = target_pos if seat is None else seat.get_center()

        agent = MovementAgent(
            agent_pos[0],
            agent_pos[1],
            0,
            radius=0.22,
            epsilon=0.1,
            box_length=2,
            box_width=1.5,
            dt=0.1,
        )

        if seat is not None:
            seat.is_occupied = True
            agent.is_sitting = True
            agent.mounting_point = seat.mounting_point
        else:
            standing_positions.append(agent_pos)
            vehicle.update_standing_attractiveness(standing_positions)

        agents.append(agent)
    return agents


def init_current_passengers_and_settle(vehicle: SimSpace, count):
    agents = init_current_passengers(vehicle, count)

    doors_as_walls = [d.get_line() for d in vehicle.doors]
    other_walls = vehicle.get_collision_wall_segments()
    all_walls = np.array(doors_as_walls + other_walls)

    settle_time = 1
    dt = 0.1

    for t in range(int(settle_time / dt)):
        for a in agents:
            a.update(all_walls, agents)
            a.target = [a.x, a.y]
    return agents


bus = articulated_bus()

passengers = init_current_passengers_and_settle(bus, 100)

ax = plt.gca()
bus.draw(ax)

ax.set_aspect("equal")

for p in passengers:
    ellipse = Ellipse((p.x, p.y), p.radius * 2, p.radius * 2)
    ax.add_patch(ellipse)

plt.show()
