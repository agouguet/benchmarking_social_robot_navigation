from polygon.nav_polygon import NavPolygon


class NavSquare(NavPolygon):
    def __init__(self, center, side_length):
        half_side = side_length / 2
        x, y = center
        vertices = [(x - half_side, y - half_side),
                    (x + half_side, y - half_side),
                    (x + half_side, y + half_side),
                    (x - half_side, y + half_side)]
        super().__init__(vertices)

    @property
    def center(self):
        minx, miny, maxx, maxy = self.bounds
        return ((maxx + minx) / 2, (maxy + miny) / 2)
    
    @property
    def side_length(self):
        minx, miny, maxx, maxy = self.bounds
        return (maxx - minx)