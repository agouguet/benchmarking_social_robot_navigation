from PyQt5.QtWidgets import*
from PyQt5.QtCore import QObject, pyqtSignal, pyqtProperty
from PyQt5 import QtWidgets, QtCore

from matplotlib.backends.backend_qt5agg import FigureCanvas, FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.backend_bases import MouseButton

from matplotlib.figure import Figure

from mbsn.model.graph.GraphState import GraphState

import networkx as nx
import numpy as np

import math, copy

from ui.mbsn_threads import *

class Human():
    def __init__(self, path):
        self._path = path
        self._node = self._path.pop(0)
        self._history = [self._node]

    def next_state(self):
        if len(self._path) > 0:
            next_node = self._path.pop(0)
            self._node = next_node
            self._history.append(next_node)
        else:
            self._history.append(self._node)

    def previous_state(self):
        prev_node = self._history.pop(len(self._history)-1)
        self._node = self._history[-1]
        self._path.insert(0, prev_node)

    @property
    def node(self):
        return self._node

    @node.setter
    def node(self, node):
        self._node = node

    @property
    def path(self):
        return [self._node] + self._path


class GraphModel(QObject):
    graph_created = pyqtSignal(nx.Graph)
    graph_updated = pyqtSignal(nx.Graph)
    state_updated = pyqtSignal(GraphState)

    def __init__(self, parent=None):
        super(GraphModel, self).__init__(parent)
        self._G = nx.Graph()
        self._state = GraphState(None, None, None)
        self._humans = []
        self._robot_path_traveled = []
        self._history = []
    
    def new_graph(self, G):
        self._G = G
        self.graph_created.emit(self._G)

    def reset(self):
        while len(self._robot_path_traveled) > 1:
            self.previous_state()
        # self.history = []
        # self.state = reset_state
        # self.robot_path = [self.state.robot_node]
        # self._humans = reset_humans

    def previous_state(self):
        if len(self._robot_path_traveled) > 1:
            self._history.pop()
            prev_node = self._robot_path_traveled.pop()
            for h in self._humans:
                h.previous_state()
            self.state = self._history.pop()

    def next_state(self, action):
        self._robot_path_traveled.append(action._node)
        for h in self._humans:
            h.next_state()
        return self.update_state(next_node=action._node)

    def update_state(self, next_node=None, next_goal=None):
        if next_node is None:
            next_node = self._state.robot_node
        if next_goal is None:
            next_goal = self._state.goal
        occupied_nodes = [0 for n in self._G.nodes]
        for h in self._humans:
            occupied_nodes[h.node] = 1
        return GraphState(next_node, next_goal, occupied_nodes)

    def add_human_path(self, start, end):
        self._history = []
        short_path = nx.shortest_path(self._G, source=start, target=end, weight=math.dist(self._G.nodes[start]['pos'],self._G.nodes[end]['pos']))
        self._humans.append(Human(short_path))
        self.state = self.update_state()

    def is_state_valid(self):
        return self._state is not None and self._state.robot_node is not None and self._state.goal is not None

    @pyqtProperty(nx.Graph, notify=graph_updated)
    def G(self):
        return self._G

    @G.setter
    def G(self, G):
        self._G = G
        self.graph_updated.emit(self._G)

    @pyqtProperty(GraphState, notify=state_updated)
    def state(self):
        return self._state

    @state.setter
    def state(self, state):
        self._state = state
        if self.is_state_valid:
            self._history.append(self._state)
        self.state_updated.emit(self._state)

    @pyqtProperty(list)
    def history(self):
        return self._history

    @history.setter
    def history(self, history):
        self._history = history

    @pyqtProperty(list)
    def robot_path(self):
        return self._robot_path_traveled
    
    @robot_path.setter
    def robot_path(self, path):
        self._robot_path_traveled = path

    @pyqtProperty(list)
    def humans(self):
        return self._humans

    @humans.setter
    def humans(self, humans):
        self._humans = humans


    def change_robot_node(self, node):
        self.history = []
        self.state = GraphState(node, self.state.goal, self.state.humans_node)
        self._robot_path_traveled = [node]

    def change_goal_node(self, node):
        self.history = []
        self.state = GraphState(self.state.robot_node, node, self.state.humans_node)

    def get_humans_path(self):
        hp = []
        for h in self._humans:
            hp.append(h._history)
        return hp