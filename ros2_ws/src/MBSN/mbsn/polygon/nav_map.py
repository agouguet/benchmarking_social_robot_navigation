from collections import defaultdict
from math import ceil, cos, radians, sin, sqrt
import matplotlib
from matplotlib import pyplot as plt
import numpy as np
from shapely import GeometryCollection, MultiPolygon, Point, Polygon, box
from mbsn.utils.map_loader import get_map_config, load_map_and_segmented_map, load_map_as_polygon
from mbsn.polygon.nav_hexagon import NavHexagon
from mbsn.polygon.nav_polygon import NavPolygon
import cv2

from mbsn.polygon.nav_room import NavRoom
from mbsn.utils.util import image_to_polygon, scale_polygon
from mbsn.polygon.visibility import VisibilityPolygon

POURCENTAGE_AREA_OF_POLYGON_ACCEPTABLE = 0.1


class NavMap(NavPolygon):

    def __init__(self, scenario, type='hexagon'):
        self.scenario = scenario
        vertices, polygon, _ = load_map_as_polygon(scenario, scale=True)
        
        super().__init__(polygon)
        self.rooms = self.get_rooms()
        if type == 'hexagon':
            # self.grid = self.create_grid_of_hexagon(polygon=polygon, hexagon_size=0.5)
            self.grid = self.create_grid_of_hexagon(polygon=polygon, hexagon_size=0.5)
        elif type == 'square':
            self.grid = self.create_grid_of_squares(polygon=polygon, square_size=1.0)
        
        self.visibility_polygon = VisibilityPolygon(vertices, polygon)

        # self.test_grid = self.create_test_grid()

        self.test_grid = self.create_test_grid_v2(polygon)
        

    def get_rooms(self, tolerance=1.0):
        scenario = self.scenario
        rooms={}
        _, map_config = get_map_config(scenario)
        _, map, segmented_map = load_map_and_segmented_map(scenario)
        for i in np.unique(segmented_map)[1:-1]:
            mask = (segmented_map == i) * np.uint8(1)
            room = cv2.bitwise_and(map, map, mask= mask)
            room = cv2.cvtColor(room.copy(), cv2.COLOR_BGR2GRAY)
            _, poly, _ = image_to_polygon(room)
            poly = poly.simplify(tolerance, preserve_topology=True)
            poly = scale_polygon(poly, map_config)
            if poly.is_valid:
                room = NavRoom(poly)
                rooms[room.id] = room

        neighbors_pair = []
        for id1, room1 in rooms.items():
            for id2, room2 in rooms.items():
                if id1 != id2 and (id1, id2) not in neighbors_pair:
                    if room1.intersects(room2):
                        neighbors_pair.append((id1, id2))
                        neighbors_pair.append((id2, id1))
                        room1.add_neighbor(room2)
                        room2.add_neighbor(room1)
        return rooms
    
    def create_grid_of_squares(self, polygon=None, square_size=10.0):
        if polygon is None:
            polygon = self.polygon
        minx, miny, maxx, maxy = polygon.bounds
        
        squares = {}
        square_dict = {}
        
        x = minx
        while x < maxx:
            y = miny
            while y < maxy:
                square_polygon = box(x, y, x + square_size, y + square_size)
                if polygon.intersects(square_polygon):
                    square = NavPolygon(square_polygon)
                    squares[square.id] = square
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
    
    def create_grid_of_hexagon(self, polygon=None, hexagon_size=10.0):
        if polygon is None:
            polygon = self.polygon

        minx, miny, maxx, maxy = polygon.bounds
        
        hexagons = []
        hexagon_dict = defaultdict(list)
        grid = {}

        hex_width = hexagon_size * 2
        hex_height = hexagon_size * 3 ** 0.5

        xo = hexagon_size * (1 + cos(radians(60)))  # X offset
        yo = hexagon_size * sin(radians(60)) * 2  # Y offset

        minx = ceil((minx - xo) / xo)
        maxx = ceil((maxx + xo) / xo)
        miny = ceil((miny - yo) / yo)
        maxy = ceil((maxy + yo) / yo)

        angle = 2 * 3.14159 / 6

        x = minx
        while x < maxx:
            y = miny
            while y < maxy:
                hex_x = x * hex_width * 0.75
                hex_y = y * hex_height + (x % 2) * hex_height / 2
                # hexagon_polygon = NavHexagon((hex_x, hex_y), hexagon_size)
                hexagon_polygon = Polygon([(hex_x + hexagon_size * cos(angle * i), hex_y + hexagon_size * sin(angle * i)) for i in range(6)])

                if polygon.buffer(0).intersects(hexagon_polygon):
                    inter = polygon.intersection(hexagon_polygon)
                    if isinstance(inter, MultiPolygon) or isinstance(inter, GeometryCollection):
                        for poly in inter.geoms:
                            if isinstance(poly, Polygon) and polygon.intersects(poly) and poly.area >= hexagon_polygon.area * POURCENTAGE_AREA_OF_POLYGON_ACCEPTABLE:
                                new_cell = NavPolygon(poly)
                                grid[new_cell.id] = new_cell
                                hexagons.append(new_cell)
                                hexagon_dict[(x, y)].append(new_cell)
                    elif isinstance(inter, Polygon):
                        if inter.area >= hexagon_polygon.area * POURCENTAGE_AREA_OF_POLYGON_ACCEPTABLE:
                            new_cell = NavPolygon(inter)
                            grid[new_cell.id] = new_cell
                            hexagons.append(new_cell)
                            hexagon_dict[(x, y)].append(new_cell)
                y += 1
            x += 1

        directions_odd = [
            (+1, 0), 
            (+1, +1),
            (0, +1),
            (-1, +1),
            (-1, 0),
            (0, -1)
        ]

        directions_even = [
            (+1, -1), 
            (+1, 0),
            (0, +1),
            (-1, 0),
            (-1, -1),
            (0, -1)
        ]

        for (x, y), list_hexagon in hexagon_dict.items():
            for hexagon in list_hexagon:
                directions = directions_even if x%2 == 0 else directions_odd
                for nx, ny in directions:
                    if (x+nx, y+ny) in hexagon_dict:
                        neighbors_hexagon = hexagon_dict[(x+nx, y+ny)]
                        for neighbor in neighbors_hexagon:
                            if hexagon.polygon.buffer(0.05).intersects(neighbor.polygon):
                                hexagon.add_neighbor(neighbor)
        return grid
    
    def create_test_grid(self):
        
        grid = {}
        id_to_id = defaultdict(list)
        for id, room in self.rooms.items():
            cells = room.get_cells_from_grid(self.grid)
            for cell in cells:
                poly = NavPolygon(cell.polygon)
                # id_to_id[poly.id] = cell.id
                id_to_id[cell.id].append(poly.id)
                grid[poly.id] = poly
        
        for _ ,ids in id_to_id.items():
            if len(ids) > 1:
                for id1 in ids:
                    poly1 = grid[id1]
                    for id2 in ids:
                        if id1 != id2:
                            poly2 = grid[id2]
                            if poly1.polygon.buffer(0.1).intersects(poly2.polygon):
                                poly1.add_neighbor(poly2)



        for id, poly in self.grid.items():
            for new_poly_id in id_to_id[id]:
                new_poly = grid[new_poly_id]
                for n in poly.neighbors:
                    for new_poly_n_id in id_to_id[n.id]:
                        new_poly_n = grid[new_poly_n_id]
                        if new_poly.polygon.buffer(0.1).intersects(new_poly_n.polygon):
                            new_poly.add_neighbor(new_poly_n)

        return grid
                
    def create_test_grid_v2(self, polygon, buffer_size=0.05):
        grid = {}
        id_to_id = defaultdict(list)
        
        for id, cell in self.grid.items():
            if polygon.intersects(cell.polygon):
                inter = polygon.intersection(cell.polygon)
                if isinstance(inter, MultiPolygon) or isinstance(inter, GeometryCollection):
                    for poly in inter.geoms:
                        if isinstance(poly, Polygon) and polygon.intersects(poly) and poly.area >= 0.1:
                            new_cell = NavPolygon(poly)
                            # for n in cell.neighbors:
                            #     if new_cell.polygon.buffer(buffer_size).intersects(n.polygon):
                            #         new_cell.add_neighbor(n)
                            grid[new_cell.id] = new_cell
                            id_to_id[cell.id].append(new_cell.id)
                elif isinstance(inter, Polygon) and inter.area >= 0.1:
                        new_cell = NavPolygon(inter)
                        # for n in cell.neighbors:
                        #     if new_cell.polygon.buffer(buffer_size).intersects(n.polygon):
                        #         new_cell.add_neighbor(n)
                        grid[new_cell.id] = new_cell
                        id_to_id[cell.id].append(new_cell.id)

        for _ ,ids in id_to_id.items():
            if len(ids) > 1:
                for id1 in ids:
                    poly1 = grid[id1]
                    for id2 in ids:
                        if id1 != id2:
                            poly2 = grid[id2]
                            if poly1.polygon.buffer(0.01).intersects(poly2.polygon):
                                poly1.add_neighbor(poly2)



        for id, poly in self.grid.items():
            for new_poly_id in id_to_id[id]:
                new_poly = grid[new_poly_id]
                for n in poly.neighbors:
                    for new_poly_n_id in id_to_id[n.id]:
                        new_poly_n = grid[new_poly_n_id]
                        if new_poly.polygon.buffer(0.01).intersects(new_poly_n.polygon):
                            new_poly.add_neighbor(new_poly_n)

        return grid