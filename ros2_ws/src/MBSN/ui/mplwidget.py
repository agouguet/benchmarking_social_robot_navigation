from PyQt5.QtWidgets import*
from PyQt5.QtCore import QObject, pyqtSignal, pyqtProperty
from PyQt5 import QtWidgets, QtCore

from matplotlib.backends.backend_qt5agg import FigureCanvas, FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.backend_bases import MouseButton

from matplotlib.figure import Figure

from mbsn.model.graph.GraphState import GraphState
from ui.mbsngraph import MBSNGraph

import networkx as nx
import numpy as np

import math, copy

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
        # print("PATH=", self._path)

    def previous_state(self):
        prev_node = self._history.pop(len(self._history)-1)
        self._node = self._history[-1]
        self._path.insert(0, prev_node)
        # print("PATH=", self._path)

    @property
    def node(self):
        return self._node

    @node.setter
    def node(self, node):
        self._node = node


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
            # self._history.append(copy.deepcopy(self))
            # self._history.append([self._robot_path_traveled, self._humans])
        self.state_updated.emit(self._state)

    @pyqtProperty(list)
    def history(self):
        return self._history

    def change_robot_node(self, node):
        self._history = []
        self.state = GraphState(node, self.state.goal, self.state.humans_node)
        self._robot_path_traveled = [node]

    def change_goal_node(self, node):
        self._history = []
        self.state = GraphState(self.state.robot_node, node, self.state.humans_node)

    def get_robot_path(self):
        return self._robot_path_traveled

    def get_humans_path(self):
        hp = []
        for h in self._humans:
            hp.append(h._history)
        return hp


class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        super(MplCanvas, self).__init__(Figure(figsize=(width, height), dpi=dpi))
        self.setParent(parent)
        self.axes = self.figure.add_subplot(111)

class GraphMplWidget(QWidget):
    updated = pyqtSignal(str)
    
    def __init__(self, parent = None):
        QWidget.__init__(self, parent)
        self.setParent(parent)

        self.canvas = MplCanvas(self)
        self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.canvas.setFocus()

        self.toolbar = NavigationToolbar(self.canvas, self)

        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(self.toolbar)
        vertical_layout.addWidget(self.canvas)

        self.setLayout(vertical_layout)

        self.graph = None

    def set_model(self, model):
        self.model = model
        self.model.graph_created.connect(self.new_graph)

    def new_graph(self, G):
        self.canvas.axes.cla()
        self.canvas.axes.patch.set_edgecolor('black')
        self.canvas.axes.patch.set_linewidth(1)
        self.graph = MBSNGraph(self.model, ax=self.canvas.axes)

    def set_select_type_node(self, str_type):
        self.graph.node_type_to_place = str_type

    def get_networkx_graph_from_interactive_graph(self):
        node_positions = self.graph.node_positions
        edges = list(self.graph.edge_artists.keys())

        G = nx.Graph()

        G.add_nodes_from(list(node_positions.keys()))
        for n, p in node_positions.items():
            G.nodes[n]['pos'] = p

        G.add_edges_from(edges)

        return G
        