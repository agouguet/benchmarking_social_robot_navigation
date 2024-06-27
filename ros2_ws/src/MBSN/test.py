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

from netgraph import BaseGraph, EditableGraph, InteractiveGraph

GRAPH_DIRECTORY = "graphs/"
TIMEOUT = 0.5

# graph = "hospital"
# robot_node = 7
# goal_node = 17
# hs = [
#     [12, 13]
# ]

graph = "hospital"
robot_node = 13
goal_node = 77
hs = [
    # [12, 13]
]

# graph = "door_passing"
# robot_node = 3
# goal_node = 12
# hs = [
#     [3, 0],
#     [6, 0]
# ]

heuristic_function = getattr(mbsn.solver.heuristicfunction, "farthest_from_human")

def one_run(agent, model, reset=True):
    if reset:
        agent.qfunction = QTable(default=-1e10)
        MBSNAgentNode.reset_visits()
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
        path = one_run(agent, model)
        print(path)
        paths.append(path)
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

    mdp = MBSN(G, model.state.goal, number_of_detected_human=len(hs), distance_factor=5.0, social_factor=5.0)
    qfunction = QTable(default=-1e10)


    for action in mdp.get_actions(model.state):
        for (new_state, probability) in mdp.get_transitions(model.state, action):
            print(action, new_state, mdp.get_reward(model.state, action, new_state))


    print(heuristic_function(mdp, model.state, debug=True))

    agent = MBSNAgentMCTS(mdp, qfunction, UpperConfidenceBounds(), heuristic_function=heuristic_function)

    one_run(agent, model)

    # human_paths = []
    # for h in model.humans:
    #     human_paths.append(h.path)

    # paths = n_run(agent, model, 10)

    # edge_count = defaultdict(lambda: 0)

    # for path in paths:
    #     for i in range(1, len(path)):
    #         n1=path[i-1]
    #         n2=path[i]
    #         edge = (n1, n2)
    #         edge_count[edge]+=1

    # print(edge_count)

    # G = model.G

    # for n in G.nodes():
    #     G.add_edge(n, n)

    # for u,v in G.edges():
    #     G[u][v]['used'] = 1

    # for e, v in edge_count.items():
    #     n1, n2 = e
    #     if n1 != n2:
    #         G[n1][n2]['used'] = v

    # edge_labels = defaultdict(lambda: 0)
    # for n1, n2, d in G.edges(data=True):
    #     print(n1, n2, d)
    #     edge_labels[(n1, n2)] = d['used']

    # print(edge_labels)
    
    # print(G)
    # fig, ax = plt.subplots(figsize=(5,4))
    # pos = nx.get_node_attributes(G,'pos')
    # nx.draw(G, pos=pos, ax=ax)

    # for edge in G.edges(data='used'):
    #     nx.draw_networkx_edges(G, pos, edgelist=[edge], width=edge[2])

    # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, label_pos=0.4,
    #                          font_color='black', font_size=12)

    # edge_width={(u,v):weight for u,v,weight in G.edges(data='used')}
    # graph = MBSNGraph(model, G, edge_width=edge_width, ax=ax)
    # plt.show()

    

    # agent = MBSNAgentMCTS(mdp, qfunction, UpperConfidenceBounds(), heuristic_function=heuristic_function)

    # human_paths = []
    # for h in model.humans:
    #     human_paths.append(h.path)

    # paths = n_run(agent, model, 5)

    # print(paths)

    

    