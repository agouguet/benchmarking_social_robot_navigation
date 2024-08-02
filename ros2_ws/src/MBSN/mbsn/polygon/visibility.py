#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys, time, math, random, numpy as np
import cv2
import yaml
from shapely.geometry import box, LineString, MultiLineString, Point, MultiPoint, Polygon, MultiPolygon, GeometryCollection
from shapely.ops import split
from shapely import polygonize, intersection, is_empty, reverse
from shapely import affinity

# from visibility import *

import matplotlib.pyplot as plt
import matplotlib
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection

from mbsn.utils.map_loader import load_map_as_polygon
from mbsn.utils.util import scale_polygon
matplotlib.use('Qt5Agg')

eps = 1e-2

maps_directory = "maps/"
scenario = "hospital"
# scenario = "test"

def distance(v1, v2):
    return ((v1.x-v2.x)**2 + (v1.y-v2.y)**2)**(1/2)

def angle_from_point(p1, p2):
    """Calculate the angle from p1 to p2."""
    import math
    return math.atan2(p2.y - p1.y, p2.x - p1.x)

def get_static_obstacles_polygons(map):
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

def get_bounds(polygon: Polygon):
    points = list(polygon.exterior.coords)

    max_x = max(points, key=lambda p: p[0])[0]
    min_x = min(points, key=lambda p: p[0])[0]
    max_y = max(points, key=lambda p: p[1])[1]
    min_y = min(points, key=lambda p: p[1])[1]

    return min_x, max_x, min_y, max_y

def visible(polygon: Polygon, pointA: Point, pointB: Point, eps=eps):
    points = list(polygon.exterior.coords)
    ray = LineString([pointA,pointB])
    inter = intersection(polygon, ray, grid_size=eps)

    # print(pointA, pointB, inter)

    if not is_empty(inter) :
        if isinstance(inter, Point):
            if not inter.equals_exact(pointA, eps) and not inter.equals_exact(pointB, eps):
                return False
        elif isinstance(inter, MultiPoint):
            if len(inter.geoms) != 2 or pointA not in inter.geoms or pointB not in inter.geoms:
                return False
        elif isinstance(inter, LineString):
            if not inter.equals_exact(ray, eps) and not inter.equals_exact(reverse(ray), eps):
                p1, p2 = inter.boundary.geoms
                if p1.distance(p2) > eps:
                    return False
        elif isinstance(inter, MultiLineString):
            outcoords = [list(i.coords) for i in inter.geoms]
            points = list(dict.fromkeys(sorted([i for sublist in outcoords for i in sublist])))
            # print(pointA, pointB, inter, points, "\n\n")
            for i in range(len(points)):
                if not Point(points[i]).equals_exact(pointA, eps) and not Point(points[i]).equals_exact(pointB, eps):
                    visi = visible(polygon, Point(points[i]), pointA)
                    if not visi:
                        return False
        elif isinstance(inter, GeometryCollection):
            return False
        else:
            return False

    return True


class VisibilityPolygon():

    def __init__(self, vertices, polygon):
        self.vertices = vertices
        self.polygon = polygon
        self.visibility_polygon = None
        self.agent = None
        self.rays = []

    def visible(self, pointA: Point, pointB: Point):
        return visible(self.polygon, pointA, pointB) #or visible(self.nav_polygon, pointB, pointA)

    def build(self, origin_x, origin_y, range=None):
        self.agent = Point(origin_x, origin_y)
        t = time.time()
        self.visible_vertices = [v for v in self.vertices if self.visible(self.agent, v)]

        self.min_x, self.max_x, self.min_y, self.max_y = get_bounds(self.polygon)
        self.bounding_box = box(self.min_x, self.min_y, self.max_x, self.max_y)

        rays = {}
        for point in self.visible_vertices:
            angle = angle_from_point(self.agent, Point(point))
            if angle not in rays:
                rays[angle] = []
            rays[angle].append(point)

        # Sort rays by angle
        rays = dict(sorted(rays.items()))
        
        intersections = []
        points_by_ray = {}

        for angle, points_ray in rays.items():
            # print(angle, points_ray)
            points_by_ray[angle] = []
            for point in points_ray:
                ray_end = point
                if self.agent != ray_end:
                    ray = self.draw_ray(self.agent, ray_end)
                    if ray is not None:
                        inter = ray.intersection(self.polygon)

                        if not is_empty(inter):
                            if isinstance(inter, LineString):
                                a, b = inter.boundary.geoms
                                if not a.equals_exact(ray_end, 1e-2) and not a.equals_exact(self.agent, 1e-2):
                                    points_by_ray[angle].append(a)
                                elif not b.equals_exact(ray_end, 1e-2) and not b.equals_exact(self.agent, 1e-2):
                                    points_by_ray[angle].append(b)
                            elif isinstance(inter, MultiLineString):
                                points = [p for line in inter.geoms for p in line.boundary.geoms]
                                points_filtered = []

                                for p1 in points:
                                    if not any(p2.equals_exact(p1, 1e-2) for p2 in points_filtered) and not p1.equals_exact(self.agent, 1e-2) and not p1.equals_exact(ray_end, 1):
                                        points_filtered.append(p1)

                                points_filtered = list(dict.fromkeys(sorted(points_filtered, key=lambda p: (self.real_distance(self.agent, p)))))
                                
                                farthest_visible_point = None

                                for p in points_filtered:
                                    if self.visible(self.agent, p):
                                        farthest_visible_point = p
                                    else:
                                        break
                                
                                if farthest_visible_point is not None:
                                    points_by_ray[angle].append(farthest_visible_point)
                
                points_by_ray[angle].append(ray_end)

        # print("Time: {}".format(time.time()-t))
        # print("\n\n")

        for angle, p in points_by_ray.items():
            points = p
            # print(angle, points)
            if len(intersections) == 0:
                points = sorted(points, key=lambda p: (-self.real_distance(self.agent, p)))
                last_points = points_by_ray[max(points_by_ray)]
                visi_between_first_and_last = False

                for p in sorted(last_points, key=lambda p: (-self.real_distance(self.agent, p))):
                    visi_between_first_and_last = self.visible(points[0], p)

                if visi_between_first_and_last:
                    for p in sorted(points, key=lambda p: (-self.real_distance(self.agent, p))):
                        intersections.append(p)
                else:
                    for p in sorted(points, key=lambda p: (self.real_distance(self.agent, p))):
                        intersections.append(p)
            else:
                points = sorted(points, key=lambda p: (-self.real_distance(self.agent, p)))
                
                if self.visible(intersections[-1], points[0]):
                    for p in points:
                        intersections.append(p)
                else:
                    for p in sorted(points, key=lambda p: (self.real_distance(self.agent, p))):
                        intersections.append(p)
        
        # print("Time: {}".format(time.time()-t))
        # print("\n\n")
        # for p in intersections:
        #     print(p.x, p.y)
        self.visibility_polygon = Polygon([[p.x, p.y] for p in intersections])

        if range is not None:
            self.circle_range = self.agent.buffer(range)
            self.visibility_polygon = self.visibility_polygon.buffer(0).intersection(self.circle_range)

        # print("Time: {}".format(time.time()-t))
        return self.agent, self.visibility_polygon

    
    def draw_ray(self, pointA, pointB):
        l = LineString([pointA, pointB])
        a, b = l.boundary.geoms

        if a.x == b.x:  # vertical line
            sliced_line_by_A = split(LineString([(a.x, self.min_y), (a.x, self.max_y)]), a.buffer(0.0001))
            for line in sliced_line_by_A.geoms:
                if line.distance(b) < 1e-8:
                    return line
        elif a.y == b.y:  # horizonthal line
            sliced_line_by_A = split(LineString([(self.min_x, a.y), (self.max_x, a.y)]), a.buffer(0.0001))
            for line in sliced_line_by_A.geoms:
                if line.distance(b) < 1e-8:
                    return line
        else:
            # linear equation: y = k*x + m
            k = (b.y - a.y) / (b.x - a.x)
            m = a.y - k * a.x
            y0 = k * self.min_x + m
            y1 = k * self.max_x + m
            x0 = (self.min_y - m) / k
            x1 = (self.max_y - m) / k
            points_on_boundary_lines = [Point(self.min_x, y0), Point(self.max_x, y1),
                                        Point(x0, self.min_y), Point(x1, self.max_y)]

            points_sorted_by_distance = sorted(points_on_boundary_lines, key=self.bounding_box.distance)
            line = LineString(points_sorted_by_distance[:2])
            sliced_line_by_A = split(line, a.buffer(0.0001))
            for line2 in sliced_line_by_A.geoms:
                if line2.distance(b) < 1e-8:
                    return line2
            return line

    def plot(self, axes):

        axes.scatter(self.agent.x, self.agent.y, color="red", s=10)

        for v in self.vertices:
            axes.scatter(v.x, v.y, color="black", s=5)

        for r in self.rays:
            axes.plot(*r.xy, color="red", alpha=0.8)

        if self.visibility_polygon is not None:
            x, y = self.visibility_polygon.exterior.xy
            axes.fill(x, y, alpha=0.3, fc='g', ec='black')

        axes.imshow(self.map, alpha=0.5)

    def real_distance(self, v1, v2):
        return distance(v1, v2)

def main():
    with open(maps_directory + scenario + "/map.yaml", 'r') as file:
        map_config = yaml.safe_load(file)
        image_map = cv2.imread(maps_directory + scenario + "/" + map_config["image"])

        fig = plt.figure()
        axes = fig.add_subplot(111)

        # axes.imshow(image_map)
        # vertices, polygon, _ = load_map_as_polygon(scenario, scale=False)
        # VP = VisibilityPolygon(vertices, polygon)
        # _, nav_graph = VP.build(700, 700)

        # nav_graph = scale_polygon(nav_graph, map_config)


        axes.imshow(image_map, extent=[-35,35,35,-35])
        vertices, polygon, _ = load_map_as_polygon(scenario, scale=True)
        VP = VisibilityPolygon(vertices, polygon)
        _, nav_graph = VP.build(0.0, 0.0)


        x, y = polygon.exterior.xy
        axes.fill(x, y, alpha=0.3, fc='k', ec='black')

        for v in vertices:
            axes.scatter(v.x, v.y, color="k")

        x, y = nav_graph.exterior.xy
        axes.fill(x, y, alpha=0.3, fc='r', ec='black')
        
        # nav_graph = scale_polygon(nav_graph, map_config)
        # x, y = nav_graph.exterior.xy
        # axes.fill(x, y, alpha=0.3, fc='g', ec='black')
        # nav_graph.plot(axes)

        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
    
        axes.axis('off')
        axes.set_aspect('equal', adjustable='box')
        # axes.invert_yaxis()
        # plt.xlim(400, 950)
        # plt.ylim(1350, 300)
        plt.show()

if __name__ == "__main__":
    main()
