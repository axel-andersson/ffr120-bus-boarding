from vehicle_definitions import articulated_bus
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

bus = articulated_bus()

rects = bus.standing_area_rectangles()
nodes = [n[2].tolist() for n in bus.inside_waypoints.get_area_waypoints()]


for d in bus.doors:
    inside = d.get_inside_waypoint()
    nodes.append(inside)

    for wp in d.get_outside_waypoints():
        nodes.append(wp)


ax = plt.gca()
bus.draw(ax)
#bus.draw_technical(ax)


for rect in rects:
    r = Rectangle(
        rect[0],
        rect[1][0] - rect[0][0],
        rect[1][1] - rect[0][1],
        fc="#00000000",
        ec="#00ff00",
    )
    ax.add_patch(r)


for node in nodes:

    pos = np.array(nodes)
    x = pos.T[0]
    y = pos.T[1]

    ax.scatter(x, y, marker="x", s=40, fc="#ff0000")

ax.set_aspect("equal")
ax.set_ylim(-1, 3)
plt.gcf().set_size_inches(32, 8)
plt.show()
