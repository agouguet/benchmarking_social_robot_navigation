#!/usr/bin/python3
# -*- coding: utf-8 -*-

import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage import util
from skimage.color import rgb2gray
import networkx as nx
import yaml

BLACK_PIXEL = np.array([10,10,10])
MAX_DEPTH = 10000


def angle(p1, p2):
    return math.atan2( p2[1] - p1[1], p2[0] - p1[0] ) * ( 180 / math.pi )

def visible(image, p1, p2):
        ang = math.radians(-angle(p1, p2))
        for depth in range(MAX_DEPTH):
            target_x = p1[1] - math.sin(ang) * depth
            target_y = p1[0] + math.cos(ang) * depth

            if target_x < 0 or target_y < 0 or target_x >= len(image) or target_y >= len(image[0]):
                return False

            col = round(target_x)
            row = round(target_y)

            if col == p1[1] and row == p1[0]:
                continue

            pixel = image[col][row]

            if (pixel <= BLACK_PIXEL).all():
                return False
            if (pixel == image[p2[1]][p2[0]]).all() and col == p2[1] and row == p2[0]:
                return True
        return False

class VisibilityGraph():
    def __init__(self, map_path, graph):
        self.graph = graph
        self.map_info = yaml.safe_load(open(map_path + "map.yaml", "rb"))
        self.map = cv2.imread(map_path + self.map_info['image'])
        self.create()

    def create(self):
        self.visibility_graph = {}
        pixels_nodes = self.nodes_to_pixel()
        for n1 in self.graph.nodes():
            self.visibility_graph[n1] = []
            for n2 in self.graph.nodes():
                if n1 != n2 and visible(self.map, pixels_nodes[n1], pixels_nodes[n2]):
                    self.visibility_graph[n1].append(n2)
        
    def nodes_to_pixel(self):
        resolution = self.map_info['resolution']
        origin = np.abs(np.array(self.map_info['origin']))/resolution
        pixels_nodes = {}
        for n in self.graph.nodes():
            x_node, y_node = self.graph.nodes(data=True)[n]['pos']
            x_pixel = origin[0] + (x_node/resolution)
            y_pixel = origin[1] + (y_node/resolution)
            pixels_nodes[n] = [int(x_pixel), int(y_pixel)]
        return pixels_nodes
    