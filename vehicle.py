import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Ellipse
from geometry import point_line_intersects, point_to_segment_distance
import numpy as np
import heapq


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
        self.is_open = False
        self.x_vec = 1
        self.y_vec = 0

        self.allow_in = True
        self.allow_out = True

    def draw(self, ax: plt.Axes):
        x0 = self.x - self.x_vec * self.width / 2
        x1 = self.x + self.x_vec * self.width / 2
        y0 = self.y - self.y_vec * self.width / 2
        y1 = self.y + self.y_vec * self.width / 2

        color = "#ceffc9" if self.is_open else "#6c0909"
        ax.plot([x0, x1], [y0, y1], color=color, lw=2)

    def get_inside_waypoint(self):
        WP_DIST = 0.3
        pos = np.array([self.x, self.y])
        vec = np.array([self.x_vec, self.y_vec])

        a = np.array([[0, -1], [1, 0]])
        return pos + WP_DIST * a @ vec

    def get_outside_waypoints(self):
        WP_DIST = 1
        pos = np.array([self.x, self.y])
        vec = np.array([self.x_vec, self.y_vec])

        a = np.array([[0, 1], [-1, 0]])
        center_waypoint = pos + WP_DIST * a @ vec
        side_waypoint_1 = center_waypoint - self.width * vec / 2
        side_waypoint_2 = center_waypoint + self.width * vec / 2
        return [center_waypoint, side_waypoint_1, side_waypoint_2]

    def draw_technical(self, ax: plt.Axes):
        outside_wps = self.get_outside_waypoints()
        inside_wp = self.get_inside_waypoint()

        ax.plot(outside_wps[0][0], outside_wps[0][1], "x", ms=8)
        ax.plot(outside_wps[1][0], outside_wps[1][1], "x", ms=8)
        ax.plot(outside_wps[2][0], outside_wps[2][1], "x", ms=8)
        ax.plot(inside_wp[0], inside_wp[1], "x", ms=8)


class VehicleWalls:
    """
    Helper class for outer walls of vehicle
    """

    def __init__(self, points):

        if len(points) != 4:
            raise "Need exactly 4 points for rectangle walls"

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
        self.is_occupied = False

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
            (self.x + self.width / 2, self.y + self.height / 2),
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


# Weights to tweak balance for standing spot attractiveness
ATTR_NEUTRAL_LEVEL = 0.2
ATTR_BASE_MODIFIER = 1
ATTR_REPULSION_MODIFIER = ATTR_NEUTRAL_LEVEL + ATTR_BASE_MODIFIER


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
        self.overall_attractiveness = (
            ATTR_BASE_MODIFIER * attractiveness + ATTR_NEUTRAL_LEVEL
        )

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
                    attractiveness[i, j] += hr.get_attractiveness(point)

        return attractiveness.clip(0, 1)

    def passenger_repulsion(self, passenger_points):
        x_grid_size = self.x_eval_points.size
        y_grid_size = self.y_eval_points.size
        repulsion = np.zeros((x_grid_size, y_grid_size))

        CUTOFF = 0.7

        for i in range(x_grid_size):
            x = self.x_eval_points[i]
            for j in range(y_grid_size):
                y = self.y_eval_points[j]
                eval_pt = np.array([x, y])
                for pass_pt in passenger_points:
                    d = np.linalg.norm(eval_pt - pass_pt)

                    if d > CUTOFF:
                        continue

                    d_rel = d / CUTOFF

                    repulsion[i, j] += np.exp(-3 * d_rel**2)

        return repulsion.clip(0, 1)

    def set_attractiveness(self, passenger_points):
        x_grid_size = self.x_eval_points.size
        y_grid_size = self.y_eval_points.size

        base_attractivenss = self.base_attractiveness
        repulsion = self.passenger_repulsion(passenger_points)

        overall = np.ones((x_grid_size, y_grid_size)) * ATTR_NEUTRAL_LEVEL
        overall += ATTR_BASE_MODIFIER * base_attractivenss
        overall -= ATTR_REPULSION_MODIFIER * repulsion
        overall = np.clip(overall, 0, ATTR_NEUTRAL_LEVEL + ATTR_BASE_MODIFIER)
        self.overall_attractiveness = overall

    def draw_attractiveness(self, ax):

        gc = self.grid_courseness

        max_value = 3

        attractiveness = self.overall_attractiveness

        for i in range(self.x_eval_points.size):
            x0 = np.max([self.x_eval_points[i] - 0.5 * gc, self.x])
            x1 = np.min([self.x_eval_points[i] + 0.5 * gc, self.x + self.width])
            for j in range(self.y_eval_points.size):
                y0 = np.max([self.y_eval_points[j] - 0.5 * gc, self.y])
                y1 = np.min([self.y_eval_points[j] + 0.5 * gc, self.y + self.height])

                pt_attractiveness = attractiveness[i][j]
                opacity = np.min([pt_attractiveness / max_value, 1])
                color = (1, 0, 0, opacity)

                rect = Rectangle((x0, y0), x1 - x0, y1 - y0, fc=color)

                ax.add_patch(rect)

    def get_rect(self):
        return [[self.x, self.y], [self.x + self.width, self.y + self.height]]


class InsideWaypoints:
    def __init__(self, standing_areas, doors: list["VehicleDoor"]):
        self.standing_areas = standing_areas
        section_waypoints = self.get_area_waypoints()

        section_count = len(standing_areas)
        door_count = len(doors)

        # Using looping and lists since n is small
        codes = []
        coordinates = []
        codes_per_section = [[] for _ in range(section_count)]
        door_codes = []
        sections_by_code = []

        for i, swp in enumerate(section_waypoints):
            code = self.section_node_code(swp, i)
            codes.append(code)
            coordinates.append(swp[2])
            codes_per_section[swp[0]].append(code)
            codes_per_section[swp[1]].append(code)
            sections_by_code.append([swp[0], swp[1]])

        for i, door in enumerate(doors):
            code = self.door_node_code(i)
            codes.append(code)
            coord = door.get_inside_waypoint()
            coordinates.append(coord)

            section = self.get_point_standing_area(coord)
            codes_per_section[section[0]].append(code)
            sections_by_code.append([section[0]])
            door_codes.append(code)

        self.codes = codes
        self.coordinates = coordinates
        self.codes_per_section = codes_per_section
        self.door_codes = door_codes

        # Create adjacency graph
        adj = [[] for _ in range(len(codes))]

        # Handle section nodes
        for i in range(len(codes)):
            code = codes[i]
            base_in_sections = sections_by_code[i]

            # See which other codes have exactly 1 section match
            # (1 match == sharing one section)
            # (2 matches == between same 2 sections)
            for j in range(len(codes)):
                if i == j:
                    continue

                compare_in_sections = sections_by_code[j]

                match_count = 0
                for section in base_in_sections:
                    if section in compare_in_sections:
                        match_count += 1

                if match_count == 1:
                    distance = float(np.linalg.norm(coordinates[i] - coordinates[j]))
                    adj[i].append([j, distance])

        self.adjacency = adj

    def section_node_code(self, swp, index):
        return f"section:{swp[0]}-{swp[1]}#{index}"

    def door_node_code(self, index):
        return f"door:inside#{index}"

    def get_area_boundary_lines(self):
        sas = self.standing_areas
        areas = list(map(lambda v: v.get_rect(), sas))

        lines = []
        for i_area in range(len(areas)):
            for j_area in range(i_area + 1, len(areas)):

                area_1 = areas[i_area]
                area_2 = areas[j_area]

                x0_1, y0_1 = area_1[0]
                x1_1, y1_1 = area_1[1]
                x0_2, y0_2 = area_2[0]
                x1_2, y1_2 = area_2[1]

                x_overlap = max(x0_1, x0_2) <= min(x1_1, x1_2)
                y_overlap = max(y0_1, y0_2) <= min(y1_1, y1_2)

                # Top edge match
                if y1_1 == y0_2 and x_overlap:
                    y = y1_1
                    x_start = max(x0_1, x0_2)
                    x_end = min(x1_1, x1_2)
                    segment = np.array([[x_start, y], [x_end, y]])
                    lines.append((i_area, j_area, segment))
                    continue

                # Bottom edge match
                if y0_1 == y1_2 and x_overlap:
                    y = y0_1
                    x_start = max(x0_1, x0_2)
                    x_end = min(x1_1, x1_2)
                    segment = np.array([[x_start, y], [x_end, y]])
                    lines.append((i_area, j_area, segment))
                    continue

                # Right edge match
                if x1_1 == x0_2 and y_overlap:
                    x = x1_1
                    y_start = max(y0_1, y0_2)
                    y_end = min(y1_1, y1_2)
                    segment = np.array([[x, y_start], [x, y_end]])
                    lines.append((i_area, j_area, segment))
                    continue

                # Left edge matchs
                if x0_1 == x1_2 and y_overlap:
                    x = x0_1
                    y_start = max(y0_1, y0_2)
                    y_end = min(y1_1, y1_2)
                    segment = np.array([[x, y_start], [x, y_end]])
                    lines.append((i_area, j_area, segment))
                    pass
        return lines

    def get_area_waypoints(self):
        area_boundary_lines = self.get_area_boundary_lines()

        MAX_WAYPOINT_SPACE = 0.3
        waypoints = []
        for i, j, segment in area_boundary_lines:
            p0 = segment[0]
            p1 = segment[1]
            segment_length = np.linalg.norm(p1 - p0)

            if segment_length <= 2 * MAX_WAYPOINT_SPACE:
                point = (p0 + p1) / 2
                waypoints.append((i, j, point))
                continue

            subdivision_count = int(np.ceil(segment_length / MAX_WAYPOINT_SPACE))
            x_space = np.linspace(p0[0], p1[0], subdivision_count + 1)
            y_space = np.linspace(p0[1], p1[1], subdivision_count + 1)

            x_points = x_space[1:-1]
            y_points = y_space[1:-1]
            points = np.array([x_points, y_points]).T

            for k in range(points.shape[0]):
                point = points[k]
                waypoints.append((i, j, point))

        return waypoints

    def get_point_standing_area(self, point):
        for i in range(len(self.standing_areas)):
            sa = self.standing_areas[i]

            # Outside x?
            if point[0] < sa.x or point[0] > sa.x + sa.width:
                continue

            # Outside y?
            if point[1] < sa.y or point[1] > sa.y + sa.height:
                continue

            # Inside
            return (i, sa)

        # No match
        return None

    def all_paths(self, start_index):

        graph = self.adjacency

        # Implementation of Dijkstra's with path lists
        n = len(graph)
        distances = [float("inf")] * n
        previous = [None] * n
        distances[start_index] = 0

        pq = [(0, start_index)]  # (distance, node)

        while pq:
            current_dist, u = heapq.heappop(pq)

            if current_dist > distances[u]:
                continue

            for v, weight in graph[u]:
                new_dist = current_dist + weight
                if new_dist < distances[v]:
                    distances[v] = new_dist
                    previous[v] = u
                    heapq.heappush(pq, (new_dist, v))

        # Build full paths for every node
        paths = []
        for end in range(n):
            path = []
            at = end
            while at is not None:
                path.append(at)
                at = previous[at]
            path.reverse()

            # If unreachable, return empty path
            if path[0] != start_index:
                paths.append([])
            else:
                paths.append(path)

        return distances, paths

    def pathfind_exit(self, start_position):
        start_segment = self.get_point_standing_area(start_position)
        start_global_node_codes = self.codes_per_section[start_segment[0]]
        start_indices = [self.codes.index(v) for v in start_global_node_codes]

        exit_doors = self.door_codes
        exit_door_indices = [self.codes.index(v) for v in exit_doors]

        shortest_distance = float("inf")
        shortest_path = []
        overall_nearest_door = 0

        for i in start_indices:
            base_distances, paths = self.all_paths(i)
            start_node_pos = self.coordinates[i]
            extra_distance = np.linalg.norm(np.array(start_position) - start_node_pos)

            total_distances = np.array(base_distances) + extra_distance

            door_distances = []
            for j in exit_door_indices:
                door_dist = total_distances[j]
                door_distances.append(door_dist)

            min_distance_door = exit_door_indices[int(np.argmin(door_distances))]
            min_distance = total_distances[min_distance_door]
            min_path = paths[min_distance_door]

            if min_distance < shortest_distance:
                shortest_path = min_path
                overall_nearest_door = min_distance_door

        coordinates = [self.coordinates[i] for i in shortest_path]

        return coordinates, overall_nearest_door


#
#
#


class SimSpace:
    """
    Helper class for handling state of simulation space
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

        waypoints = InsideWaypoints(standing_areas, doors)
        self.inside_waypoints = waypoints

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

    def draw_technical(self, ax):
        for door in self.doors:
            door.draw_technical(ax)
        for area in self.standing_areas:
            area.draw_attractiveness(ax)

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

    def update_standing_attractiveness(self, points):
        for sa in self.standing_areas:
            sa.set_attractiveness(points)

    #
    # Pathfinding methods
    #

    def select_free_seat(self):
        free_seats = list(filter(lambda s: not s.isOccupied, self.seats))
        # seat is selected at random with equal prob.
        # Since mounting is done from aisle, priority for
        # window seats has no impact

        seat_index = int(np.random.randint(len(free_seats)))
        return free_seats[seat_index]

    def select_standing_position(self):
        standing_areas = self.standing_areas

        # Randomly select a position that is within
        # a range of the maximum attractiveness of
        # all standing positions

        max_attractiveness = -float("inf")

        for sa in standing_areas:
            attr = sa.overall_attractiveness
            sa_max_attr = np.max(attr)
            if sa_max_attr > max_attractiveness:
                max_attractiveness = sa_max_attr

        positions = []
        for sa in standing_areas:
            base_x = sa.x
            base_y = sa.y
            gc = sa.grid_courseness

            attr = sa.overall_attractiveness

            max_indices = np.argwhere(attr == max_attractiveness).T
            points_x = max_indices[0] * gc + base_x + gc / 2
            points_y = max_indices[1] * gc + base_y + gc / 2

            coordinates = np.array([points_x, points_y]).T.tolist()
            positions += coordinates

        index = int(np.random.randint(len(positions)))
        return positions[index]

    def get_path_out(self, start_position):
        """
        Gets path from start position

        Returns a list of waypoints to the door,
        as well as the index of the door.
        """

        return self.inside_waypoints.pathfind_exit(start_position)
