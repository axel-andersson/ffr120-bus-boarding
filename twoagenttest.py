import time
from tkinter import *
import numpy as np
from agent import MovementAgent  # din klass

window_size = 600
tk = Tk()
tk.geometry(f"{window_size + 20}x{window_size + 20}")
tk.configure(background="#000000")
canvas = Canvas(tk, background="#ECECEC")
tk.attributes("-topmost", 0)
canvas.place(x=10, y=10, height=window_size, width=window_size)

# --- väggar
walls = np.array(
    [
        [[0.0, 0.0], [0.0, 0.0]]
        # [[0.0, 0.0], [10.0, 0.0]],
        # [[0.0, 2.0], [10.0, 2.0]],
    ]
)


# --- agents
agent1 = MovementAgent(
    x=0.0,
    y=-2.0,
    angle=0.0,
    radius=0.3,
    epsilon=0.1,
    box_length=3,
    box_width=1.5,
    dt=0.1,
)

agent2 = MovementAgent(
    x=0.0,
    y=1.0,
    angle=0.0,
    radius=0.3,
    epsilon=0.1,
    box_length=3,
    box_width=1.5,
    dt=0.1,
)

agent3 = MovementAgent(
    x=0.0,
    y=-1.0,
    angle=0.0,
    radius=0.3,
    epsilon=0.1,
    box_length=2,
    box_width=1.5,
    dt=0.1,
)

agent4 = MovementAgent(
    x=0.0, y=-3, angle=0.0, radius=0.3, epsilon=0.1, box_length=3, box_width=1.5, dt=0.1
)

targets = np.array(
    [
        [5.0, 0.0],
        [3.11745, 3.90915],
        [-1.11260, 4.87464],
        [-4.50485, 2.16942],
        [-4.50485, -2.16942],
        [-1.11260, -4.87464],
        [3.11745, -3.90915],
    ]
)

agent1.target_queue = targets
agent2.target_queue = targets
agent3.target_queue = targets
agent4.target_queue = targets

agents = [agent1, agent2, agent3, agent4]



# start vel
for ag in agents:
    ag.v = 0.1

# --- world → screen mapping ---
world_min_x, world_max_x = -10.0, 20.0
world_min_y, world_max_y = -10.0, 10.0


def world_to_screen(x, y):
    sx = (x - world_min_x) / (world_max_x - world_min_x) * window_size
    sy = window_size - (y - world_min_y) / (world_max_y - world_min_y) * window_size
    return sx, sy


def world_radius_to_pixels(r):
    return r / (world_max_x - world_min_x) * window_size


def draw_scene():
    canvas.delete("all")
    # draw ticket check point (radius 1)
    tx, ty = world_to_screen(agent1.ticket_check_pos[0], agent1.ticket_check_pos[1])
    ticket_r = world_radius_to_pixels(1)

    canvas.create_oval(
        tx - ticket_r,
        ty - ticket_r,
        tx + ticket_r,
        ty + ticket_r,
        outline="green",
        width=2,
    )

    # draw walls
    for w in walls:
        (x1, y1), (x2, y2) = w
        sx1, sy1 = world_to_screen(x1, y1)
        sx2, sy2 = world_to_screen(x2, y2)
        canvas.create_line(sx1, sy1, sx2, sy2, fill="black", width=3)

    # draw targets
    for ag in agents:
        ax, ay = world_to_screen(ag.target[0], ag.target[1])
        r_att = 4
        canvas.create_oval(
            ax - r_att, ay - r_att, ax + r_att, ay + r_att, outline="red", width=2
        )

    # draw agents
    for i, ag in enumerate(agents):
        sx, sy = world_to_screen(ag.x, ag.y)
        r_pix = world_radius_to_pixels(ag.radius)

        canvas.create_oval(sx - r_pix, sy - r_pix, sx + r_pix, sy + r_pix, fill="blue")

        canvas.create_text(sx, sy - 10, text=str(i + 1), fill="black")


T_STEPS = 600
step = 0


def sim_step():
    global step
    if step >= T_STEPS:
        return

    for ag in agents:
        ag.update(walls, agents)

    draw_scene()
    step += 1
    tk.after(50, sim_step)


# start
draw_scene()
tk.after(50, sim_step)
tk.mainloop()
