import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Ellipse
from geometry import point_line_intersects
import numpy as np


#
# Vehicle geometry
#
class VehicleDoor:
    """
    Helper class for doors for vehicle
    """

    def __init__(self, id: str, x: float, y: float, width: float):
        """
        Create vehicle door

        :param id: string id for door
        :type id: str
        :param position: Center position of door [x, y]
        :param width: Width of door [m]
        :type width: float
        """

        self.id = id
        self.x = x
        self.y = y
        self.width = width
        self.isOpen = False
        self.x_vec = 1
        self.y_vec = 0

    def draw(self, ax: plt.Axes):
        x0 = self.x - self.x_vec * self.width / 2
        x1 = self.x + self.x_vec * self.width / 2
        y0 = self.y - self.y_vec * self.width / 2
        y1 = self.y + self.y_vec * self.width / 2

        color = "#ceffc9" if self.isOpen else "#6c0909"
        ax.plot([x0, x1], [y0, y1], color=color, lw=2)


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

    def cut_out_door(self, door: VehicleDoor):

        door_point = np.array([door.x, door.y])

        # Check for intersections
        intersection_index = None
        for i, segment in enumerate(self.wall_segments):
            if point_line_intersects(door_point, segment):
                intersection_index = i
                break

        if intersection_index == None:
            raise "Door center doesn't intersect a wall segment."

        # Find points for new segments
        segment_before = self.wall_segments[intersection_index - 1]
        segment_after = self.wall_segments[
            (intersection_index + 1) % len(self.wall_segments)
        ]

        start_point = np.array(segment_before[1])
        end_point = np.array(segment_after[0])

        wall_delta = end_point - start_point
        wall_vector = wall_delta / np.linalg.norm(wall_delta)

        # Update door vector with wall info
        door.x_vec = wall_vector[0]
        door.y_vec = wall_vector[1]

        first_door_end = door_point - wall_vector * door.width / 2
        second_door_end = door_point + wall_vector * door.width / 2

        # Check if door ends also overlap wall segment.
        if not point_line_intersects(
            first_door_end, self.wall_segments[intersection_index]
        ):
            raise "Door end doesn't intersect a wall segment."

        if not point_line_intersects(
            second_door_end, self.wall_segments[intersection_index]
        ):
            raise "Door end doesn't intersect a wall segment."

        # Get new segments and replace
        first_wall_segment = [start_point.tolist(), first_door_end.tolist()]
        second_wall_segment = [second_door_end.tolist(), end_point.tolist()]

        self.wall_segments.pop(intersection_index)

        # Inserts happen BEFORE specified indices
        self.wall_segments.insert(intersection_index, second_wall_segment)
        self.wall_segments.insert(intersection_index, first_wall_segment)

    def draw(self, ax: plt.Axes):
        for segment in self.wall_segments:
            x0 = segment[0][0]
            x1 = segment[1][0]
            y0 = segment[0][1]
            y1 = segment[1][1]
            ax.plot([x0, x1], [y0, y1], color="#a0a0a0", lw=3)


class PassengerSeat:
    """
    Helper class for a single passenger seat
    """

    def __init__(
        self,
        x,
        y,
        width,
        height,
    ):
        """
        Docstring for __init__

        :param x: min x seat coordinate
        :param y: min y seat coordinate
        :param width: seat size in the x-direction
        :param height: seat size in the y-direction
        :param center_point: center point of seat [x, y]
        """
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def draw(self, ax: plt.Axes):
        rect = Rectangle(
            (self.x, self.y), self.width, self.height, fc="#bcbcbc", ec="#3A3A3A"
        )
        ellipse = Ellipse(
            (self.x + self.width / 2, self.y + self.width / 2),
            self.width*0.9,
            self.height*0.9,
            fc="#dadada",
        )
        ax.add_patch(rect)
        ax.add_patch(ellipse)


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


class Vehicle:
    """
    Helper class for handling state of vehicle
    """

    def __init__(self, points):
        pass


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
door = VehicleDoor("test", 7, 0, 2)
seat1 = PassengerSeat(0, 0, 1, 1)

vw.cut_out_door(door)
door.isOpen = True
ax = plt.gca()
ax.set_aspect("equal")

vw.draw(ax)
door.draw(ax)
seat1.draw(ax)

plt.show()
