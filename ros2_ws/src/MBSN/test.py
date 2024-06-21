import os, sys, math, random, copy, yaml, cv2, warnings, pickle, json, numpy as np, networkx as nx
from collections import defaultdict
from PyQt5.QtWidgets import*
from PyQt5.uic import loadUi
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)
from shapely.geometry import box, LineString, MultiLineString, Point, MultiPoint, Polygon, MultiPolygon, GeometryCollection

from mbsn.model.graph.MBSN import MBSN
from mbsn.utils.visibility_graph import VisibilityGraph
from mbsn.solver.MCTS import MBSNAgentMCTS, MBSNAgentNode
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.solver.qtable import QTable
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.utils.graph_visualisation import GraphVisualisation
from mbsn.model.graph.GraphState import GraphState
from mbsn.model.graph.GraphAction import GraphAction

import matplotlib.pyplot as plt
import mbsn.solver.heuristicfunction

from ui.mbsn_canvas import *
from ui.model import *
from ui.mbsn_threads import *

GRAPH_DIRECTORY = "graphs/"
TIMEOUT = 0.5
graph = "mini_door_passing"
robot_node = 1
goal_node = 6
hs = [
    [3, 0]
]

heuristic_function = getattr(mbsn.solver.heuristicfunction, "farthest_from_human")

def one_run(agent, model):
    while 1:
        node, _ = agent.mcts(model.state, timeout=TIMEOUT)
        if node.mdp.is_terminal(node.state):
            break
        action, value = node.get_value()
        model.state = model.next_state(action)
    return [n for n in model.robot_path]

def n_run(agent, model, n):
    paths = []
    print('\rRun '+str(0)+"/"+str(n), end='')
    for i in range(n):
        paths.append(one_run(agent, model))
        model.reset()
        print('\rRun '+str(i+1)+"/"+str(n), end='')
    print("\nDone.")
    return paths


with open(GRAPH_DIRECTORY  + graph + ".pickle", 'rb') as file:
    G = pickle.load(file)

    model = GraphModel()
    model.state = GraphState(None, None, [0 for n in G.nodes])
    model.change_robot_node(robot_node)
    model.change_goal_node(goal_node)
    model.new_graph(G)

    for h in hs:
        model.add_human_path(h[0], h[1])

    mdp = MBSN(G, model.state.goal, number_of_detected_human=len(hs))
    qfunction = QTable(default=-1e10)

    heuristic_function(mdp, model.state)


    
    # agent = MBSNAgentMCTS(mdp, qfunction, UpperConfidenceBounds(), heuristic_function=heuristic_function)

    # human_paths = []
    # for h in model.humans:
    #     human_paths.append(h.path)

    # paths = n_run(agent, model, 5)

    # print(paths)

    

    