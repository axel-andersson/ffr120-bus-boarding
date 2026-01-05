from vehicle import (
    VehicleWalls,
    SimSpace,
    VehicleDoor,
    PassengerSeat,
    Obstacle,
    Handrail,
)
import matplotlib.pyplot as plt
import numpy as np


# Loosely modeled on the 18 m Volvo 7900 Electric
# as used by Västtrafik in Gothenburg, Sweden
def articulated_bus():
    bus_width = 2.5
    bus_length = 18

    #
    # Walls and doors
    #
    walls = VehicleWalls(
        [[0, 0], [bus_length, 0], [bus_length, bus_width], [0, bus_width]]
    )

    d1 = VehicleDoor(17, 0, 1.3)
    d2 = VehicleDoor(11, 0, 1.3)
    d3 = VehicleDoor(4.8, 0, 1.3)

    #
    # Seats
    #
    sw = 0.45  # Seat width
    sd = 0.8  # Seat depth, Including foot space
    y_mount = bus_width / 2  # Mounting point for seats

    srw_y = 0  # Seat right, window
    sra_y = sw  # Seat right, aisle
    sla_y = bus_width - 2 * sw  # Seat left, aisle
    slw_y = bus_width - sw  # Seat left, window

    def get_full_row(x, n):
        mp = [x + sd / 2, y_mount]
        return [
            PassengerSeat(f"{n}-rw", [x, srw_y], sd, sw, mp),
            PassengerSeat(f"{n}-ra", [x, sra_y], sd, sw, mp),
            PassengerSeat(f"{n}-la", [x, sla_y], sd, sw, mp),
            PassengerSeat(f"{n}-lw", [x, slw_y], sd, sw, mp),
        ]

    # Rows 1-4, between front 2 doors
    front_seats = (
        get_full_row(15.2, 1)
        + get_full_row(14.4, 2)
        + get_full_row(13.6, 3)
        + get_full_row(12.8, 4)
    )

    # Between back 2 doors, articulated section
    extra_seat = PassengerSeat("7-ra", [5.7, srw_y], sd, sw, [5.7 + sd / 2, y_mount])
    middle_seats = get_full_row(9.3, 5) + get_full_row(8.5, 6) + [extra_seat]

    # After back door
    last_mp = [1.4 * sd, y_mount]
    last_row = [
        PassengerSeat("12-rw", [0, srw_y], sd, sw, last_mp),
        PassengerSeat("12-ra", [0, sra_y], sd, sw, last_mp),
        PassengerSeat("12-aa", [0, sra_y + sw], sd, sw, last_mp),
        PassengerSeat("12-la", [0, sla_y], sd, sw, last_mp),
        PassengerSeat("12-lw", [0, slw_y], sd, sw, last_mp),
    ]

    back_seats = (
        get_full_row(3.2, 8)
        + get_full_row(2.4, 9)
        + get_full_row(1.6, 10)
        + get_full_row(0.8, 11)
        + last_row
    )

    all_seats = front_seats + middle_seats + back_seats

    #
    # Obstacles
    #

    driver_cabin = Obstacle([16, sla_y], 2, 2 * sw)
    articulation_left = Obstacle([6.5, slw_y], 2, sw)
    articulation_right = Obstacle([6.5, srw_y], 2, sw)

    obstacles = [driver_cabin, articulation_left, articulation_right]

    # Add obstacle to fill out back row if needed
    if 5 * sw < bus_width:
        filler_obstacle = Obstacle([0, sra_y + 2 * sw], sd, bus_width - 5 * sw)
        obstacles += [filler_obstacle]
    #
    # Handrails
    #

    # Front standing area
    front_handrails = [
        Handrail([10.1, 2.5], [12.8, 2.5]),
        Handrail([10.1, 2], [12.8, 2]),
        Handrail([10.1, 1.5], [12.8, 1.5]),
    ]

    # Middle standing area
    middle_handrails = [
        Handrail([7, 2], [8, 2]),
        Handrail([6.5, 0.5], [8, 0.5]),
    ]

    # Back standing area
    back_handrails = [
        Handrail([4, 2.5], [6.5, 2.5]),
        Handrail([4, 2], [6, 2]),
        Handrail([4.2, 1.5], [5.5, 1.5]),
    ]

    handrails = front_handrails + middle_handrails + back_handrails

    ss = SimSpace(walls, [d1, d2, d3], all_seats, obstacles, handrails)
    return ss


"""

ss = articulated_bus()
ss.doors[0].allow_out = False
ss.doors[1].allow_out = False
ss.doors[2].allow_in = False

start = [11, -2]
end = [2.5, 1.25]

path_in = ss.get_path_out(end, start)
print(path_in)
p = np.concat(
    (np.array([end]), np.array(path_in[0]), np.array(path_in[1]), np.array([start]))
)

wss = ss.get_collision_wall_segments()

ax = plt.gca()

ss.draw(ax)
ss.draw_technical(ax)
ax.set_aspect("equal")

plt.show()

"""
