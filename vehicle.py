import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Ellipse
from geometry import point_line_intersects, point_to_segment_distance
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

        self.segments = segments

    def cut_out_door(self, door: VehicleDoor):

        door_point = np.array([door.x, door.y])

        # Check for intersections
        intersection_index = None
        for i, segment in enumerate(self.segments):
            if point_line_intersects(door_point, segment):
                intersection_index = i
                break

        if intersection_index == None:
            raise "Door center doesn't intersect a wall segment."

        # Find points for new segments
        segment_before = self.segments[intersection_index - 1]
        segment_after = self.segments[(intersection_index + 1) % len(self.segments)]

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
        if not point_line_intersects(first_door_end, self.segments[intersection_index]):
            raise "Door end doesn't intersect a wall segment."

        if not point_line_intersects(
            second_door_end, self.segments[intersection_index]
        ):
            raise "Door end doesn't intersect a wall segment."

        # Get new segments and replace
        first_wall_segment = [start_point.tolist(), first_door_end.tolist()]
        second_wall_segment = [second_door_end.tolist(), end_point.tolist()]

        self.segments.pop(intersection_index)

        # Inserts happen BEFORE specified indices
        self.segments.insert(intersection_index, second_wall_segment)
        self.segments.insert(intersection_index, first_wall_segment)

    def draw(self, ax: plt.Axes):
        for segment in self.segments:
            x0 = segment[0][0]
            x1 = segment[1][0]
            y0 = segment[0][1]
            y1 = segment[1][1]
            ax.plot([x0, x1], [y0, y1], color="#a0a0a0", lw=3)


class PassengerSeat:
    """
    Helper class for a single passenger seat
    """

    def __init__(self, id, position, width, height, mounting_point):
        """
        Docstring for __init__

        :parm id: id for identifying seat
        :param position: min coordinates [x, y]
        :param width: seat size in the x-direction
        :param height: seat size in the y-direction
        :param mounting_point: point where passengers enter and exit chair [x, y]
        """
        self.id = id
        self.x = position[0]
        self.y = position[1]
        self.width = width
        self.height = height

        self.mounting_point = mounting_point
        self.isOccupied = False

    def draw(self, ax: plt.Axes, with_path_finding=False):

        if with_path_finding:
            mounting_point = Ellipse(self.mounting_point, 0.2, 0.2, fc="#f540ff68")
            ax.add_patch(mounting_point)
            ax.plot(
                [self.mounting_point[0], self.x + self.width / 2],
                [self.mounting_point[1], self.y + self.width / 2],
                color="#f540ff68",
                lw=1,
            )

        rect = Rectangle(
            (self.x, self.y), self.width, self.height, fc="#bcbcbc", ec="#3A3A3A"
        )
        ellipse = Ellipse(
            (self.x + self.width / 2, self.y + self.width / 2),
            self.width * 0.9,
            self.height * 0.9,
            fc="#dadada",
        )
        ax.add_patch(rect)
        ax.add_patch(ellipse)


class Obstacle:
    """
    Helper class for space-taking obstacles
    """

    def __init__(self, position, width, height):
        """
        Docstring for __init__

        :param position: Position of min x and y
        :param width: size in x
        :param height: size in y
        """

        self.x = position[0]
        self.y = position[1]
        self.width = width
        self.height = height

    def draw(self, ax):
        rect = Rectangle(
            (self.x, self.y), self.width, self.height, fc="#343434", ec="#0A0A0A"
        )
        ax.add_patch(rect)


class Handrail:
    """
    Handrails define where in the standing areas agents prefer to stand.
    """

    def __init__(self, start, end):
        self.start_x = start[0]
        self.start_y = start[1]
        self.end_x = end[0]
        self.end_y = end[1]

    def get_segment(self):
        return np.array([[self.start_x, self.start_y], [self.end_x, self.end_y]])

    def get_attractiveness(self, point):
        segment = self.get_segment()
        d = point_to_segment_distance(point, segment)

        CUTOFF = 0.75
        if d > CUTOFF:
            return 0

        # Arbitrairy formula to make passengers prefer to stand near handrails
        # in range 1 to 0
        return np.sqrt((CUTOFF - d) / CUTOFF)

    def draw(self, ax: plt.Axes):
        ax.plot(
            [self.start_x, self.end_x],
            [self.start_y, self.end_y],
            "--",
            color="#fff820",
            lw=1,
        )


class StandingArea:
    """
    Docstring for StandingArea
    """

    def __init__(self, rect, handrails):
        self.x = rect[0][0]
        self.y = rect[0][1]
        self.width = rect[1][0] - rect[0][0]
        self.height = rect[1][1] - rect[0][1]

        self.grid_courseness = 0.2  # Courseness of attractiveness grid
        x_eval_points, y_eval_points = self.init_position_ranges()
        self.x_eval_points = x_eval_points
        self.y_eval_points = y_eval_points

        attractiveness = self.init_base_attractiveness(handrails)
        self.base_attractiveness = attractiveness

    def init_position_ranges(self):
        min_x = self.x
        max_x = self.x + self.width

        min_y = self.y
        max_y = self.y + self.height

        gc = self.grid_courseness

        x_start = np.floor((min_x) / gc) * gc
        y_start = np.floor((min_y) / gc) * gc

        x_end = np.floor(max_x / gc) * gc
        y_end = np.floor(max_y / gc) * gc

        x_range = np.arange(x_start, x_end, gc) + 0.5 * gc
        y_range = np.arange(y_start, y_end, gc) + 0.5 * gc

        return x_range, y_range

    def init_base_attractiveness(self, handrails):
        x_grid_size = self.x_eval_points.size
        y_grid_size = self.y_eval_points.size
        attractiveness = np.zeros((x_grid_size, y_grid_size))

        for i in range(x_grid_size):
            x = self.x_eval_points[i]
            for j in range(y_grid_size):
                y = self.y_eval_points[j]
                point = np.array([x, y])
                for hr in handrails:
                    attractiveness[i, j] = hr.get_attractiveness(point)

        return attractiveness.clip(0, 1)

    def draw_attractiveness(self, ax):

        gc = self.grid_courseness

        max_value = 3

        for i in range(self.x_eval_points.size):
            x0 = np.max([self.x_eval_points[i] - 0.5 * gc, self.x])
            x1 = np.min([self.x_eval_points[i] + 0.5 * gc, self.x + self.width])
            for j in range(self.y_eval_points.size):
                y0 = np.max([self.y_eval_points[j] - 0.5 * gc, self.y])
                y1 = np.min([self.y_eval_points[j] + 0.5 * gc, self.y + self.height])

                attractiveness = self.base_attractiveness[i][j]
                opacity = np.min([attractiveness / max_value, 1])
                color = (1, 0, 0, opacity)

                rect = Rectangle((x0, y0), x1 - x0, y1 - y0, fc=color)

                ax.add_patch(rect)


#
#
#


class Vehicle:
    """
    Helper class for handling state of vehicle
    """

    def __init__(self, walls, doors, seats, obstacles, handrails):
        self.walls = walls
        for door in doors:
            walls.cut_out_door(door)
        self.doors = doors
        self.seats = seats
        self.obstacles = obstacles
        self.handrails = handrails

        standing_area_rects = self.standing_area_rectangles()
        standing_areas = [StandingArea(rect, handrails) for rect in standing_area_rects]
        self.standing_areas = standing_areas

    def draw(self, ax):
        self.walls.draw(ax)

        for door in self.doors:
            door.draw(ax)

        for seat in self.seats:
            seat.draw(ax)

        for obstacle in self.obstacles:
            obstacle.draw(ax)

        for handrail in self.handrails:
            handrail.draw(ax)

        for sa in self.standing_areas:
            sa.draw_attractiveness(ax)

    def delimiting_coordinates(self):
        x = []
        y = []

        # walls
        segments = self.walls.segments
        for segment in segments:
            x.append(segment[0][0])
            x.append(segment[1][0])

            y.append(segment[0][1])
            y.append(segment[1][1])

        # seats
        for seat in self.seats:
            x.append(seat.x)
            x.append(seat.x + seat.width)
            y.append(seat.y)
            y.append(seat.y + seat.height)

        # rectangular obstacles
        for obstacle in self.seats:
            x.append(obstacle.x)
            x.append(obstacle.x + obstacle.width)
            y.append(obstacle.y)
            y.append(obstacle.y + obstacle.height)

        unique_x = np.unique(x)
        unique_y = np.unique(y)

        return unique_x, unique_y

    def area_rectangles(self):
        delimiting_x, delimiting_y = self.delimiting_coordinates()
        rectangles = []
        for i in range(delimiting_x.size - 1):
            for j in range(delimiting_y.size - 1):
                x0 = delimiting_x[i]
                x1 = delimiting_x[i + 1]
                y0 = delimiting_y[j]
                y1 = delimiting_y[j + 1]
                rectangles.append([[x0, y0], [x1, y1]])
        return rectangles

    def free_rectangle_fragments(self):
        all_rectangles = self.area_rectangles()

        filtered_rectangles = []
        # Check if any part of seat intersects rectangle
        for i in range(len(all_rectangles)):
            rect = all_rectangles[i]
            rect_min_x = rect[0][0]
            rect_max_x = rect[1][0]
            rect_min_y = rect[0][1]
            rect_max_y = rect[1][1]

            has_intersection = False
            for seat in self.seats:
                seat_min_x = seat.x
                seat_max_x = seat.x + seat.width
                seat_min_y = seat.y
                seat_max_y = seat.y + seat.height

                x_intersect = rect_min_x < seat_max_x and rect_max_x > seat_min_x
                y_intersect = rect_min_y < seat_max_y and rect_max_y > seat_min_y

                if x_intersect and y_intersect:
                    has_intersection = True
                    break

            for obstacle in self.obstacles:
                obs_min_x = obstacle.x
                obs_max_x = obstacle.x + obstacle.width
                obs_min_y = obstacle.y
                obs_max_y = obstacle.y + obstacle.height

                x_intersect = rect_min_x < obs_max_x and rect_max_x > obs_min_x
                y_intersect = rect_min_y < obs_max_y and rect_max_y > obs_min_y

                if x_intersect and y_intersect:
                    has_intersection = True
                    break

            if not has_intersection:
                filtered_rectangles.append(rect)
        return filtered_rectangles

    def standing_area_rectangles(self):
        # assuming fragments are sorted in increasing x and then increasing y
        fragments = self.free_rectangle_fragments()

        y_merged_rects = []
        for base_rect in fragments:

            merged_rect = base_rect.copy()
            while True:
                # merge with rects with same x-coords and aligning y
                filter_fn = lambda ir: (
                    (
                        lambda index, rect: rect[0][0]
                        == base_rect[0][0]  # Left x match
                        and rect[1][0] == base_rect[1][0]  # Right x match
                        and rect[0][1] == base_rect[1][1]  # Edge-to-edge y
                    )(*ir)
                )

                matches = list(filter(filter_fn, enumerate(fragments)))
                if len(matches) == 0:
                    break

                match_index, match_rect = matches[0]
                fragments.pop(match_index)
                merged_rect[1][1] = match_rect[1][1]

            y_merged_rects.append(merged_rect)

        x_merged_rects = []
        for base_rect in y_merged_rects:

            merged_rect = base_rect.copy()
            while True:
                # merge with rects with same y-coords and aligning x
                filter_fn = lambda ir: (
                    (
                        lambda index, rect: rect[0][1]
                        == base_rect[0][1]  # Bottom y match
                        and rect[1][1] == base_rect[1][1]  # Top y match
                        and rect[0][0] == base_rect[1][0]  # Edge-to-edge x
                    )(*ir)
                )

                matches = list(filter(filter_fn, enumerate(y_merged_rects)))
                if len(matches) == 0:
                    break

                match_index, match_rect = matches[0]
                y_merged_rects.pop(match_index)
                merged_rect[1][0] = match_rect[1][0]

            x_merged_rects.append(merged_rect)
        return x_merged_rects


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
seat1 = PassengerSeat(0, [4.1, 0], 1, 1, [0.5, 2.5])
seat2 = PassengerSeat(1, [4.1, 1], 1, 1, [0.5, 2.5])
seat3 = PassengerSeat(0, [4.1, 3], 1, 1, [0.5, 2.5])
seat4 = PassengerSeat(1, [4.1, 4], 1, 1, [0.5, 2.5])

obs = Obstacle([8, 2], 2, 3)
hr = Handrail([1, 0], [1, 5])

v = Vehicle(vw, [door], [seat1, seat2, seat3, seat4], [obs], [hr])

ax = plt.gca()
ax.set_aspect("equal")

v.draw(ax)
rects = v.standing_area_rectangles()
"""

for r in rects:
    print(r)
    vis_rect = Rectangle(r[0], r[1][0] - r[0][0], r[1][1] - r[0][1], ec="black")
    ax.add_patch(vis_rect)

"""

plt.show()
