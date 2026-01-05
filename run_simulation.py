from real_time_visualization import draw_scene, setup_win
from vehicle_definitions import articulated_bus
from vehicle import SimSpace
from agent import MovementAgent
import numpy as np


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
            box_length=1.5,
            box_width=0.7,
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
            a.reached_final_target = True

    a.v = 0

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
            box_length=1.5,
            box_width=1,
            dt=0.1,
        )
        agents.append(agent)

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
    # Make them walk further away
    target_pos[1] -= platform_rect[1][1] - platform_rect[0][1]

    # Get two-parted exit path
    exit_path = vehicle.get_path_out(pathfind_start_point, target_pos)
    passenger.pre_hold_target_queue = exit_path[0]
    passenger.post_hold_target_queue = exit_path[1]


def prime_random_exiting_passengers(
    vehicle: SimSpace, all_passengers, platform_rect, count
):
    indices = np.random.choice(list(range(len(all_passengers))), count)

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


def prime_entering_passenger(
    vehicle: SimSpace, agent: MovementAgent, current_passengers
):
    seat, target_pos = vehicle.find_boarding_target()

    enter_path = vehicle.get_path_in([agent.x, agent.y], target_pos)
    agent.pre_hold_target_queue = enter_path[0]
    agent.post_hold_target_queue = enter_path[1]
    agent.target_seat = seat

    if seat is not None:
        seat.is_occupied = True
    else:
        current_pos = [[p.x, p.y] for p in current_passengers]
        vehicle.update_standing_attractiveness(current_pos + [target_pos])

    # Adjust agent position to be closer to doors
    # This simulates people usually entering through the closest door

    CLOSENESS_WEIGHT = 2
    x = (CLOSENESS_WEIGHT * enter_path[0][0][0] + agent.x) / (CLOSENESS_WEIGHT + 1)

    agent.x = x
    agent.target = [x, agent.y]


def prime_all_entering_passengers(
    vehicle: SimSpace, entering: list["MovementAgent"], current_passengers
):
    for agent in entering:
        prime_entering_passenger(vehicle, agent, current_passengers)

    # Settle
    settle_time = 0.5
    dt = 0.1

    for t in range(int(settle_time / dt)):
        for a in entering:
            a.update(np.array([]), entering)
            a.target = [a.x, a.y]
            a.reached_final_target = True
    a.v = 0

    return entering


def start_exiting_passenger(agent: MovementAgent):
    if agent.is_sitting:
        agent.x = agent.mounting_point[0]
        agent.y = agent.mounting_point[1]
        agent.is_sitting = False

    agent.is_exiting = True
    agent.target_queue = agent.pre_hold_target_queue[1:]
    agent.target = agent.pre_hold_target_queue[0]
    agent.reached_final_target = False
    agent.stress = True


def start_entering_passenger(agent: MovementAgent):

    agent.is_entering = True
    agent.target_queue = agent.pre_hold_target_queue[1:]
    agent.target = agent.pre_hold_target_queue[0]
    agent.reached_final_target = False


def visualize_bus_dynamics(
    current_passenger_count,
    exitng_passenger_count,
    entering_pasenger_count,
    front_door_only,
    check_tickets,
):

    bus = articulated_bus()
    bus.exit_priority = True  # Fixes unrealistically large clashes

    if front_door_only:
        bus.doors[0].allow_out = False
        bus.doors[1].allow_in = False
        bus.doors[2].allow_in = False

    start_passengers = init_current_passengers_and_settle(bus, current_passenger_count)
    wr = get_waiting_rectangle(bus, 1.5)

    waiting_passengers = init_waiting_passengers(wr, entering_pasenger_count)

    remaining_passengers, exiting_passengers = prime_random_exiting_passengers(
        bus, start_passengers, wr, exitng_passenger_count
    )

    for p in exiting_passengers:
        start_exiting_passenger(p)

    prime_all_entering_passengers(bus, waiting_passengers, remaining_passengers)

    if check_tickets:
        for a in waiting_passengers:
            a.must_check_ticket = True

    OPENING_STEP = 20

    # Used for final evaluation
    step_size = 0.1
    complete_step = None

    window_size, tk, canvas = setup_win()

    def sim_step(
        vehicle: SimSpace,
        entering_agents: list["MovementAgent"],
        exiting_agents,
        still_agents,
        last_step,
    ):
        step = last_step

        all_agents = entering_agents + exiting_agents + still_agents

        walls = np.array(vehicle.get_collision_wall_segments())
        
        if step == OPENING_STEP:
            vehicle.doors_open = True
            for p in waiting_passengers:
                start_entering_passenger(p)
            for agent in all_agents:
                agent.target_queue += agent.post_hold_target_queue

        for ag in all_agents:
            ag.update(walls, all_agents)

        draw_scene(
            vehicle, entering_agents, exiting_agents, still_agents, canvas, window_size
        )
        step += 1
        rich_sim_step = lambda: sim_step(
            vehicle, entering_agents, exiting_agents, still_agents, step
        )
        tk.after(100, rich_sim_step)

    # start
    draw_scene(
        bus,
        waiting_passengers,
        exiting_passengers,
        remaining_passengers,
        canvas,
        window_size,
    )
    tk.after(
        100,
        sim_step(bus, waiting_passengers, exiting_passengers, remaining_passengers, 0),
    )
    tk.mainloop()

    # Handle results

    if complete_step is None:
        print("RUN DID NOT COMPLETE")
        return

    elapsed_time = complete_step * step_size
    print("ET", elapsed_time)


def evaluate_bus_dynamics(
    current_passenger_count,
    exitng_passenger_count,
    entering_pasenger_count,
    front_door_only,
    check_tickets,
    file_name,
):

    bus = articulated_bus()
    bus.exit_priority = True  # Fixes unrealistically large clashes

    if front_door_only:
        bus.doors[0].allow_out = False
        bus.doors[1].allow_in = False
        bus.doors[2].allow_in = False

    start_passengers = init_current_passengers_and_settle(bus, current_passenger_count)
    wr = get_waiting_rectangle(bus, 1.5)

    waiting_passengers = init_waiting_passengers(wr, entering_pasenger_count)

    remaining_passengers, exiting_passengers = prime_random_exiting_passengers(
        bus, start_passengers, wr, exitng_passenger_count
    )

    for p in exiting_passengers:
        start_exiting_passenger(p)

    prime_all_entering_passengers(bus, waiting_passengers, remaining_passengers)

    if check_tickets:
        for a in waiting_passengers:
            a.must_check_ticket = True

    MAX_T_STEPS = 1000

    OPENING_STEP = 20

    # Used for run termination check
    exit_offset = 0.5
    enter_offset = 0.2

    # Used for final evaluation
    step_size = 0.1
    complete_step = None

    def sim_step(
        vehicle: SimSpace,
        entering_agents: list["MovementAgent"],
        exiting_agents,
        still_agents,
        last_step,
    ):
        step = last_step
        if step >= MAX_T_STEPS:
            return False

        walls = np.array(vehicle.get_collision_wall_segments())
        all_agents = entering_agents + exiting_agents + still_agents

        if step == OPENING_STEP:
            vehicle.doors_open = True
            for p in waiting_passengers:
                start_entering_passenger(p)
            for agent in all_agents:
                agent.target_queue += agent.post_hold_target_queue

        exiting_eval = True
        entering_eval = True
        for ag in all_agents:
            ag.update(walls, all_agents)

            # Check exiting status
            if ag.is_exiting and ag.y >= -exit_offset:
                exiting_eval = False

            # Check entering status
            if ag.is_entering and ag.y <= enter_offset:
                entering_eval = False

        if exiting_eval and entering_eval:
            nonlocal complete_step
            complete_step = step
            return False

        step += 1
        return True

    # start
    for i in range(MAX_T_STEPS):
        carry_on = sim_step(
            bus, waiting_passengers, exiting_passengers, remaining_passengers, i
        )
        if not carry_on:
            break

    # Handle results

    if complete_step is None:
        print("RUN DID NOT COMPLETE")
        return

    elapsed_time = (complete_step - OPENING_STEP) * step_size

    with open(file_name, "a") as f:
        np.savetxt(f, [elapsed_time], delimiter=",", fmt="%.6f")
