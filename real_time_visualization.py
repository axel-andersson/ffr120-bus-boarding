from tkinter import Canvas, Tk
from agent import MovementAgent
from vehicle import SimSpace


def setup_win():

    window_size = 600
    tk = Tk()
    tk.geometry(f"{window_size + 20}x{window_size + 20}")
    tk.configure(background="#000000")
    canvas = Canvas(tk, background="#ECECEC")
    tk.attributes("-topmost", 0)
    canvas.place(x=10, y=10, height=window_size, width=window_size)
    return window_size, tk, canvas


# --- world → screen mapping ---
world_min_x, world_max_x = -2.0, 20.0
world_min_y, world_max_y = -5.0, 10.0


def world_to_screen(x, y, window_size):
    sx = (x - world_min_x) / (world_max_x - world_min_x) * window_size
    sy = window_size - (y - world_min_y) / (world_max_y - world_min_y) * window_size
    return sx, sy


def world_radius_to_pixels(r, window_size):
    return r / (world_max_x - world_min_x) * window_size


def draw_scene(
    vehicle: SimSpace,
    entering_agents: list["MovementAgent"],
    exiting_agents,
    still_agents,
    canvas,
    window_size,
):
    canvas.delete("all")

    walls = vehicle.get_collision_wall_segments()

    # draw walls
    for w in walls:
        (x1, y1), (x2, y2) = w
        sx1, sy1 = world_to_screen(x1, y1, window_size)
        sx2, sy2 = world_to_screen(x2, y2, window_size)
        canvas.create_line(sx1, sy1, sx2, sy2, fill="black", width=3)

    # draw agents
    for i, ag in enumerate(entering_agents):
        sx, sy = world_to_screen(ag.x, ag.y, window_size)
        r_pix = world_radius_to_pixels(ag.radius, window_size)

        canvas.create_oval(sx - r_pix, sy - r_pix, sx + r_pix, sy + r_pix, fill="green")
        canvas.create_text(sx, sy - 10, text=str(i + 1), fill="black")

    for i, ag in enumerate(exiting_agents):
        sx, sy = world_to_screen(ag.x, ag.y, window_size)
        r_pix = world_radius_to_pixels(ag.radius, window_size)

        canvas.create_oval(sx - r_pix, sy - r_pix, sx + r_pix, sy + r_pix, fill="red")
        canvas.create_text(sx, sy - 10, text=str(i + 1), fill="black")

    for i, ag in enumerate(still_agents):
        sx, sy = world_to_screen(ag.x, ag.y, window_size)
        r_pix = world_radius_to_pixels(ag.radius, window_size)

        canvas.create_oval(sx - r_pix, sy - r_pix, sx + r_pix, sy + r_pix, fill="blue")
        canvas.create_text(sx, sy - 10, text=str(i + 1), fill="black")
