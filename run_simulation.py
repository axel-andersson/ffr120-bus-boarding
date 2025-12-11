from vehicle_definitions import articulated_bus
from vehicle import SimSpace
from agent import MovementAgent
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle
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

    settle_time = 0.5
    dt = 0.1

    for t in range(int(settle_time / dt)):
        for a in agents:
            a.update(all_walls, agents)
            a.target = [a.x, a.y]
    return agents


def get_waiting_rectangle(vehicle: SimSpace, height):
    TOP_OFFSET = 0.5  # Add offset from vehicle to simplify
    SIDE_OFFSET = 0
    min_y = float("inf")
    min_x = float("inf")
    max_x = -float("inf")

    for ws in vehicle.walls.segments:
        y = min(ws[0][1], ws[1][1])
        x0 = min(ws[0][0], ws[1][0])
        x1 = max(ws[0][0], ws[1][0])
        min_y = min(y, min_y)
        min_x = min(x0, min_x)
        max_x = max(x1, max_x)

    top_y = min_y - TOP_OFFSET
    left_x = min_x - SIDE_OFFSET
    right_x = max_x + SIDE_OFFSET
    bottom_y = min_y - TOP_OFFSET - height

    return [[left_x, bottom_y], [right_x, top_y]]


def init_waiting_passengers(rect, count):
    agents = []
    x0 = rect[0][0]
    x1 = rect[1][0]
    y0 = rect[0][1]
    y1 = rect[1][1]

    for _ in range(count):
        x = x0 + np.random.rand() * (x1 - x0)
        y = y0 + np.random.rand() * (y1 - y0)
        agent = MovementAgent(
            x,
            y,
            0,
            radius=0.22,
            epsilon=0.1,
            box_length=2,
            box_width=1.5,
            dt=0.1,
        )
        agents.append(agent)

    # Settle
    settle_time = 1
    dt = 0.1

    for t in range(int(settle_time / dt)):
        for a in agents:
            a.update(np.array([]), agents)
            a.target = [a.x, a.y]
    return agents


bus = articulated_bus()

start_passengers = init_current_passengers_and_settle(bus, 10)
wr = get_waiting_rectangle(bus, 3)
waiting_passengers = init_waiting_passengers(wr, 10)

ax = plt.gca()
bus.draw(ax)

ax.set_aspect("equal")

for p in start_passengers:
    ellipse = Ellipse((p.x, p.y), p.radius * 2, p.radius * 2, fc="#ff2adf")
    ax.add_patch(ellipse)

for p in waiting_passengers:
    ellipse = Ellipse((p.x, p.y), p.radius * 2, p.radius * 2, fc="#2f52ff")
    ax.add_patch(ellipse)

print(wr)

area_rect = Rectangle(
    wr[0], wr[1][0] - wr[0][0], wr[1][1] - wr[0][1], fc="#00000000", ec="#0000ff"
)
ax.add_patch(area_rect)

plt.show()
