from tkinter import *
from agent import MovementAgent
import numpy as np


def to_window_coordinates(r, arena_size, window_size):
    return r / arena_size * window_size + window_size / 2


def base_agent_blob(
    canvas: Canvas,
    a: MovementAgent,
    window_width,
    window_height,
    arena_width,
    arena_height,
):
    dummy_size = 0.3
    canvas.create_oval(
        to_window_coordinates(a.x - dummy_size / 2, arena_width, window_width),
        to_window_coordinates(a.y - dummy_size / 2, arena_height, window_height),
        to_window_coordinates(a.x + dummy_size / 2, arena_width, window_width),
        to_window_coordinates(a.y + dummy_size / 2, arena_height, window_height),
        outline="#00C0C0",
        fill="#00C0C0",
    )


def move_agent_blob(
    canvas: Canvas,
    blob,
    a: MovementAgent,
    window_width,
    window_height,
    arena_width,
    arena_height,
):
    dummy_size = 0.3
    canvas.coords(
        blob,
        to_window_coordinates(a.x - dummy_size / 2, arena_width, window_width),
        to_window_coordinates(a.y - dummy_size / 2, arena_height, window_height),
        to_window_coordinates(a.x + dummy_size / 2, arena_width, window_width),
        to_window_coordinates(a.y + dummy_size / 2, arena_height, window_height),
    )


def base_agent_arrow(
    canvas: Canvas,
    a: MovementAgent,
    window_width,
    window_height,
    arena_width,
    arena_height,
):

    velocity_scale = 1
    end_x = np.cos(a.angle) * a.v * velocity_scale
    end_y = np.sin(a.angle) * a.v * velocity_scale

    canvas.create_line(
        to_window_coordinates(a.x, arena_width, window_width),
        to_window_coordinates(a.y, arena_height, window_height),
        to_window_coordinates(end_x, arena_width, window_width),
        to_window_coordinates(end_y, arena_height, window_height),
        fill="#0000ff",
        width=2,
        arrow=LAST,
    )


def move_agent_arrow(
    canvas: Canvas,
    arrow,
    a: MovementAgent,
    window_width,
    window_height,
    arena_width,
    arena_height,
):
    velocity_scale = 1
    end_x = np.cos(a.angle) * a.v * velocity_scale
    end_y = np.sin(a.angle) * a.v * velocity_scale

    canvas.coords(
        arrow,
        to_window_coordinates(a.x, arena_width, window_width),
        to_window_coordinates(a.y, arena_height, window_height),
        to_window_coordinates(end_x, arena_width, window_width),
        to_window_coordinates(end_y, arena_height, window_height),
    )


def base_wall(
    canvas: Canvas,
    wall_position,
    window_width,
    window_height,
    arena_width,
    arena_height,
):
    x0 = wall_position[0][0]
    y0 = wall_position[0][1]
    x1 = wall_position[1][0]
    y1 = wall_position[1][1]

    canvas.create_line(
        to_window_coordinates(x0, arena_width, window_width),
        to_window_coordinates(y0, arena_height, window_height),
        to_window_coordinates(x1, arena_width, window_width),
        to_window_coordinates(y1, arena_height, window_height),
        width=3,
        fill="#808080"
    )


#
#
# Dummy code
#
#


#
# Dummy state
#

test_agent = MovementAgent(0, 0, 0)
test_wall = np.array([[2, 0], [2, 2]])


#
# Dummy rendering
#

window_size = 600
arena_size = 10

tk = Tk()
tk.geometry(f"{window_size + 20}x{window_size+20}")
tk.configure(background="#000000")

canvas = Canvas(tk, background="#ECECEC")
tk.attributes("-topmost", 0)
canvas.place(x=10, y=10, height=window_size, width=window_size)

# Draw dummy state
base_agent_blob(canvas, test_agent, window_size, window_size, arena_size, arena_size)
base_agent_arrow(canvas, test_agent, window_size, window_size, arena_size, arena_size)
base_wall(canvas, test_wall, window_size, window_size, arena_size, arena_size)

tk.mainloop()  # Release animation handle (close window to finish).
