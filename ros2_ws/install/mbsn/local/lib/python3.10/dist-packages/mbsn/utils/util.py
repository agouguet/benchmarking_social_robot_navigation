import heapq
from scipy.spatial import distance
from shapely.geometry import Point, Polygon, LineString, box
import os, yaml, cv2, numpy as np, matplotlib

def euclidean_distance(point1, point2):
    return distance.euclidean(point1, point2)

def midpoint(p1, p2):
    return Point((p1.x+p2.x)/2, (p1.y+p2.y)/2)

# Function to scale polygon coordinates
def scale_polygon(polygon, map_config):
    scale_factor = map_config["resolution"]
    origin = map_config["origin"]
    exterior_scaled_coords = [((x) * scale_factor + origin[0], (y) * scale_factor + origin[1]) for x, y in polygon.exterior.coords]
    
    interiors_scaled = []
    for interior in polygon.interiors:
        interior_scaled = []
        for x, y in interior.coords:
            interior_scaled.append(((x) * scale_factor + origin[0], (y) * scale_factor + origin[1]))
        interiors_scaled.append(interior_scaled)
    
    return Polygon(exterior_scaled_coords, holes=interiors_scaled)

# Function to scale list of coordinates
def scale_list_vertices(vertices, map_config):
    scale_factor = map_config["resolution"]
    origin = map_config["origin"]
    scaled_coords = [Point((v.x) * scale_factor + origin[0], (v.y) * scale_factor + origin[1]) for v in vertices]
    return scaled_coords

def image_to_polygon(map):
    if(len(map.shape)==3):
        map = cv2.cvtColor(map.copy(), cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(map[:, :], 150, 255, cv2.THRESH_BINARY)
    mask = np.invert(mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    vertices = []
    polygons = []
    for c in range(len(contours)):
        poly = []
        for p in contours[c]:
            n = tuple(p[0])
            vertices.append(Point(n))
            poly.append(n)
        if len(poly)>=4:
            polygons.append(Polygon(poly))

    outer = polygons[1]
    inners = polygons[2:]

    exterior = list(outer.exterior.coords)
    interiors = [list(inner.exterior.coords) for inner in inners]

    inners.append(polygons[0]-polygons[1])

    return vertices, Polygon(exterior, holes=interiors), inners

def random_color():
    r = np.round(np.random.rand(),1)
    g = np.round(np.random.rand(),1)
    b = np.round(np.random.rand(),1)
    a = np.round(np.clip(np.random.rand(), 0, 1), 1)
    return [r,g,b,0.7]

def generate_square_grid(polygon, square_size):
    # Get the bounds of the polygon
    minx, miny, maxx, maxy = polygon.bounds

    # Create the grid
    x_coords = np.arange(minx, maxx, square_size)
    y_coords = np.arange(miny, maxy, square_size)
    grid_squares = []

    for x in x_coords:
        for y in y_coords:
            square = box(x, y, x + square_size, y + square_size)
            if polygon.intersects(square):
                grid_squares.append(square)

    return grid_squares

def subdivide_square(square):
    minx, miny, maxx, maxy = square.bounds
    midx = (minx + maxx) / 2
    midy = (miny + maxy) / 2

    # Create 4 smaller squares
    square1 = box(minx, midy, midx, maxy)
    square2 = box(midx, midy, maxx, maxy)
    square3 = box(minx, miny, midx, midy)
    square4 = box(midx, miny, maxx, midy)

    return [square1, square2, square3, square4]

def astar(start, goal, grid):

    def heuristic(cell1, cell2):
        # Utilisation de la distance de Manhattan comme heuristique
        a = grid[cell1].centroid
        b = grid[cell2].centroid
        return abs(a.x - b.x) + abs(a.y - b.y)

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruire le chemin
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        # print(current)
        # if current == 980:
        #     print("C=", current)
        #     for poly in grid[current]:
        #         print(poly.neighbors)

        #     for neighbor in grid[current][0].neighbors:
        #         for poly in grid[current]:
        #             print("     ", neighbor.id, poly.polygon.buffer(0.02).intersects(neighbor.polygon), neighbor.id in grid)
        
        neighbors = []
        # print(current)
        # for poly in grid[current]:
        #     for n in poly.neighbors:
        #         if n not in neighbors:
        #             neighbors.append(n)
                


        for neighbor in grid[current].neighbors:
            close_enough = False
            if grid[current].polygon.buffer(0.01).intersects(neighbor.polygon) and neighbor.id in grid:
                close_enough = True

            if close_enough:
                tentative_g_score = g_score[current] + 1
                if neighbor.id not in g_score or tentative_g_score < g_score[neighbor.id]:
                    came_from[neighbor.id] = current
                    g_score[neighbor.id] = tentative_g_score
                    f_score[neighbor.id] = tentative_g_score + heuristic(neighbor.id, goal)
                    heapq.heappush(open_set, (f_score[neighbor.id], neighbor.id))

    return None  # Aucun chemin trouvé


def ros_point_to_shapely_point(ros_point):
    return Point(ros_point.x, -ros_point.y)

def shapely_point_to_ros_point(shapely_point):

    return Point(shapely_point.x, shapely_point.y)
