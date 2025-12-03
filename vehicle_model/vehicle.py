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
        pass

    def draw(ax: plt.Axes):
        pass


class VehicleDoors:
    """
    Helper class for doors for vehicle
    """

    def __init__(self, point, width):
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
