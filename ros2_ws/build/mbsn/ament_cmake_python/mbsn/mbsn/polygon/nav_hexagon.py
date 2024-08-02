
from math import cos, sin

from shapely import Point
from mbsn.polygon.nav_polygon import NavPolygon


class NavHexagon(NavPolygon):

    def __init__(self, center, size):
        """ Create a hexagon centered at `center` with a given `size`. """
        if isinstance(center, tuple):
            center = Point(center)
        angle = 2 * 3.14159 / 6
        super().__init__([
            (center.x + size * cos(angle * i), center.y + size * sin(angle * i)) 
            for i in range(6)
        ])