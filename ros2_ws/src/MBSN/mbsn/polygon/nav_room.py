from shapely import GeometryCollection, MultiPolygon, Point, Polygon, box

from .nav_polygon import NavPolygon
from mbsn.utils.util import midpoint, random_color


class NavRoom(NavPolygon):

    def __init__(self, *args, id=None):
        super().__init__(*args)
        color = random_color()
        self.color=color
        self.facecolor=color
        self.zorder = 1

    def get_cells_from_grid(self, grid, buffer_size=0.05):
        cells = []
        for id, cell in grid.items():
            if self.polygon.intersects(cell.polygon):
                inter = self.polygon.intersection(cell.polygon)
                if isinstance(inter, MultiPolygon) or isinstance(inter, GeometryCollection):
                    for poly in inter.geoms:
                        if isinstance(poly, Polygon) and self.polygon.intersects(poly) and poly.area >= 0.05: #TODO: Find half of hexagon area
                            new_cell = NavPolygon(poly, id=cell.id)
                            for n in cell.neighbors:
                                if new_cell.polygon.buffer(buffer_size).intersects(n.polygon):
                                    new_cell.add_neighbor(n)
                            # new_cell.neighbors = cell.neighbors
                            cells.append(new_cell)
                elif isinstance(inter, Polygon) and inter.area >= 0.35:
                        new_cell = NavPolygon(inter, id=cell.id)
                        for n in cell.neighbors:
                            if new_cell.polygon.buffer(buffer_size).intersects(n.polygon):
                                new_cell.add_neighbor(n)
                        # new_cell.neighbors = cell.neighbors
                        cells.append(new_cell)
        return cells
    
    def divide_into_square_mesh(self, polygon, square_size=10.0):
        minx, miny, maxx, maxy = polygon.bounds
        squares = []
        square_dict = {}
        
        x = minx
        while x < maxx:
            y = miny
            while y < maxy:
                square_polygon = box(x, y, x + square_size, y + square_size)
                if self.polygon.intersects(square_polygon):
                    square = NavPolygon(square_polygon.intersection(self.polygon))
                    squares.append(square)
                    square_dict[(x, y)] = square
                y += square_size
            x += square_size
        
        for (x, y), square in square_dict.items():
            neighbors_coords = [
                (x - square_size, y), (x + square_size, y),  # left, right
                (x, y - square_size), (x, y + square_size)   # bottom, top
            ]
            for nx, ny in neighbors_coords:
                if (nx, ny) in square_dict:
                    neighbor_square = square_dict[(nx, ny)]
                    square.add_neighbor(neighbor_square)
        
        return squares

    
    def polygon_transition_point(polygon1, polygon2, distance = 0.1):
        point_of_polygon1 = [Point(c) for c in polygon1.exterior.coords]
        point_of_polygon2 = [Point(c) for c in polygon2.exterior.coords]
        transition_points = []
        used = []

        for p1 in point_of_polygon1:
            for p2 in point_of_polygon2:
                if p1.distance(p2) <= distance:
                    transition_points.append(midpoint(p1, p2))
                    used.append(p1)
                    used.append(p2)

        for p1 in point_of_polygon1:
            if p1 not in used and p1.distance(polygon2.polygon) <= distance:
                transition_points.append(p1)
                used.append(p1)

        for p2 in point_of_polygon2:
            if p2 not in used and p2.distance(polygon1.polygon) <= distance:
                transition_points.append(p2)
                used.append(p2)

        # boundary_line = LineString(transition_points)
        # simplified_boundary = boundary_line.simplify(distance+1, preserve_topology=True)
        # transition_points = [Point(p) for p in simplified_boundary.coords]

        return transition_points

    def plot_polygon(self, ax, add_points=False, add_id=False):
        super().plot_polygon(ax, add_points=add_points, add_id=add_id)
        centroid = self.polygon.centroid
        ax.plot(centroid.x, centroid.y, 'ro', zorder=100)
        ax.text(centroid.x, centroid.y, str(self.id), fontsize=8, ha='center', va='center', color='white', bbox=dict(facecolor='red', alpha=0.5), zorder=101)
