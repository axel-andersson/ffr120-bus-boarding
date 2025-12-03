import matplotlib.pyplot as plt


class Vehicle:
    """
    Helper class for handling state of vehicle
    """

    def __init__(self, points):
        pass


#
# Vehicle geometry
#
class VehicleWalls:
    """
    Helper class for outer walls of vehicle
    """

    def __init__(self, points):

        if len(points) < 4:
            raise "Need at least 4 points for walls with right angles."

        segments = []
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i + 1]

            dx = end_point[0] - start_point[0]
            dy = end_point[1] - start_point[1]

            if dx == 0 and dy == 0:
                raise "Invalid geometry, points cannot overlap"
            if dx != 0 and dy != 0:
                raise "Invalid geometry, only vertical and horizontal segments are allowed"

            segment = [start_point, end_point]
            segments.append(segment)

        start_point = points[-1]
        end_point = points[0]

        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]

        if dx == 0 and dy == 0:
            raise "Invalid geometry, points cannot overlap"
        if dx != 0 and dy != 0:
            raise "Invalid geometry, only vertical and horizontal segments are allowed"

        segment = [start_point, end_point]
        segments.append(segment)

        self.wall_segments = segments
        self.door_segments = []

    def draw(self, ax: plt.Axes):
        for segment in self.wall_segments:
            print(segment)
            x0 = segment[0][0]
            x1 = segment[1][0]
            y0 = segment[0][1]
            y1 = segment[1][1]
            ax.plot([x0, x1], [y0, y1], color="#a0a0a0", lw=3)


class VehicleDoors:
    """
    Helper class for doors for vehicle
    """

    def __init__(self, name, position, width):
        pass

    def draw(ax: plt.Axes):
        pass


class PassengerSeat:
    """
    Helper class for a single passenger seat
    """

    def __init__(self):
        pass

    def draw(ax: plt.Axes):
        pass


class PassengerSeatUnit:
    """
    Helper class for multiple passenger seats, e.g. a seat pair
    """

    def __init__(self):
        pass

    def draw(ax: plt.Axes):
        pass


class Handrail:
    """
    Handrails define where in the standing areas agents prefer to stand.
    """

    def __init__(self):
        pass

    def draw(ax: plt.Axes):
        pass


#
#
#


class LocalNode:
    """
    Local nodes are navigation nodes that can be placed
    on a straight line segment between two other nodes and that
    only connect to nodes on that line segment, not counting seats.
    """


class GlobalNode:
    """
    Global nodes are navigation nodes that connect multiple areas.
    """


# Things that need to be solved:
# Rectangle merging
# Auto waypoint-categorization.


#
# Dummy code for testing below:
#


vw = VehicleWalls([[0, 0], [10, 0], [10, 5], [0, 5]])
print(vw.wall_segments)

ax = plt.gca()
ax.set_aspect("equal")
vw.draw(ax)
plt.show()
