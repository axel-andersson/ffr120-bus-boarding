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


def get_random_rect_point(rect):
    x0 = rect[0][0]
    x1 = rect[1][0]
    y0 = rect[0][1]
    y1 = rect[1][1]

    x = x0 + np.random.rand() * (x1 - x0)
    y = y0 + np.random.rand() * (y1 - y0)

    return [x, y]


def init_waiting_passengers(rect, count):
    agents = []

    for _ in range(count):
        pos = get_random_rect_point(rect)
        agent = MovementAgent(
            pos[0],
            pos[1],
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


def prime_exiting_passenger(
    vehicle: SimSpace, passenger: MovementAgent, all_passengers, platform_rect
):
    seats = vehicle.seats
    seat_positions = [s.get_center() for s in seats]

    passenger_seat = None
    for i in range(len(seat_positions)):
        sp = seat_positions[i]
        if sp[0] == passenger.x and sp[1] == passenger.y:
            passenger_seat = seats[i]
            break

    # Update attractiveness if standing
    if passenger_seat is None:
        pi = all_passengers.index(passenger)  # Passenger index
        remaining_standing = all_passengers[:pi] + all_passengers[pi + 1 :]

        passenger_points = [[p.x, p.y] for p in remaining_standing]
        vehicle.update_standing_attractiveness(passenger_points)

        pathfind_start_point = [passenger.x, passenger.y]
        pass
    # Dismount seat if sitting
    else:
        passenger_seat.is_occupied = False
        pathfind_start_point = passenger.mounting_point

    target_pos = get_random_rect_point(platform_rect)

    # Get two-parted exit path
    exit_path = vehicle.get_path_out(pathfind_start_point, target_pos)
    passenger.pre_hold_target_queue = exit_path[0]
    passenger.post_hold_target_queue = exit_path[1]


def prime_random_exiting_passengers(
    vehicle: SimSpace, all_passengers, platform_rect, count
):
    indices = np.random.choice(np.arange(len(all_passengers)), count)

    non_exiting = []
    exiting = []

    for i in range(len(all_passengers)):
        passenger = all_passengers[i]
        if i in indices.tolist():
            prime_exiting_passenger(vehicle, passenger, all_passengers, platform_rect)
            exiting.append(passenger)
        else:
            non_exiting.append(passenger)

    return non_exiting, exiting


def prime_entering_passenger(vehicle: SimSpace, agent, current_passengers):
    seat, target_pos = vehicle.find_boarding_target()

    enter_path = vehicle.get_path_in(target_pos)[1][0]
    if seat is not None:
        seat.is_occupied = True
    else:
        vehicle.update_standing_attractiveness(current_passengers + target_pos)

    # Adjust agent position to be closer to doors
    # This simulates people usually entering through the closest door

    CLOSENESS_WEIGHT = 4
    x = (CLOSENESS_WEIGHT * enter_path[0][0] + agent.x) / (CLOSENESS_WEIGHT + 1)
    y = (CLOSENESS_WEIGHT * enter_path[0][1] + agent.x) / (CLOSENESS_WEIGHT + 1)
    
    return [x, y]


def start_exiting_passenger(agent: MovementAgent):
    if agent.is_sitting:
        agent.x = agent.mounting_point[0]
        agent.y = agent.mounting_point[1]

    agent.is_exiting = True
    agent.target_queue = agent.pre_hold_target_queue


# TODO FOR BASE CASE
# - start exit for passenger
# - start enter for passenger
# - open doors
# - take seat

# TODO FOR EVALUATION


bus = articulated_bus()

start_passengers = init_current_passengers_and_settle(bus, 10)
wr = get_waiting_rectangle(bus, 2)
waiting_passengers = init_waiting_passengers(wr, 10)

prime_random_exiting_passengers(bus, start_passengers, wr, 3)

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
