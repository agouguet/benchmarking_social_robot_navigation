# ------------------------------------------------------
# -------------------- mplwidget.py --------------------
# ------------------------------------------------------
from PyQt5.QtWidgets import*
from PyQt5.QtCore import pyqtSignal
from PyQt5 import QtWidgets, QtCore

from matplotlib.backends.backend_qt5agg import FigureCanvas, FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.backend_bases import MouseButton

from matplotlib.figure import Figure

from netgraph import Graph
from netgraph import EditableGraph, InteractiveGraph
from netgraph._artists import NodeArtist, EdgeArtist

from mbsn.model.graph.GraphState import GraphState

import networkx as nx
import numpy as np

import math

NEUTRAL_COLOR = "#a6fbff"
ROBOT_COLOR = "#ffdaa6"
GOAL_COLOR = "#9aff75"
OCCUPIED_COLOR = "#ffa6b3"

class MyGraph(EditableGraph):

    def __init__(self, state, *args, **kwargs):
        EditableGraph.__init__(self, *args, **kwargs)
        self.set_axes_limits()

        self.state = state

        edge_labels = {x:round(math.dist(self.node_positions[x[0]], self.node_positions[x[1]]), 2) for i,x in enumerate(self.edges)}
        
        self.edge_label_fontdict = self._initialize_edge_label_fontdict(None)
        self.edge_label_position = 0.5
        self.edge_label_rotate = True
        self.edge_label_artists = dict()
        self.draw_edge_labels(edge_labels, self.edge_label_position, self.edge_label_rotate, self.edge_label_fontdict)
        
        self.node_type_to_place = "none"
        self.emphasizeable_artists = []
        self.update_color_node()

    def _on_release(self, event):
        if self._currently_clicking_on_artist is not None and not self._currently_dragging:
            closest_node = 0
            min_dist = math.inf
            for n, pos in self.node_positions.items():
                dist = math.dist(self._currently_clicking_on_artist.xy, pos)
                if dist <  min_dist:
                    closest_node = n
                    min_dist = dist
            if self.node_type_to_place == "robot":
                self.state.robot_node = closest_node #GraphState(closest_node, self.state.goal, self.state.humans_node)
            elif self.node_type_to_place == "goal":
                self.state.goal = closest_node #GraphState(self.state.robot_node, closest_node, self.state.humans_node)
            elif self.node_type_to_place == "human":
                self.state.humans_node[closest_node] = not self.state.humans_node[closest_node]

            self.update_color_node()
        super()._on_release(event)
        
    def _move(self, event):
        EditableGraph._move(self, event)
        
    def _add_node(self, event):
        EditableGraph._add_node(self, event)
        self.state.humans_node.append(0)
        
    def _update_edge_label_positions(self, edges):
        EditableGraph._update_edge_label_positions(self, edges)
        # edge_labels = {x:round(math.dist(self.node_positions[x[0]], self.node_positions[x[1]]), 2) for i,x in enumerate(self.edges)}
        # self.draw_edge_labels(edge_labels, self.edge_label_position, self.edge_label_rotate, self.edge_label_fontdict)

        xmin, xmax = self.ax.get_xlim()
        ymin, ymax = self.ax.get_ylim()

        for edge, edge_label_artist in self.edge_label_artists.items():
            x = edge_label_artist._x
            y = edge_label_artist._y
            if x > xmin and x < xmax and y > ymin and y < ymax:
                edge_label_artist.set_visible(True)
                edge_label_artist.set_text(round(math.dist(self.node_positions[edge[0]], self.node_positions[edge[1]]), 2))
            else:
                edge_label_artist.set_visible(False)


    def _update_node_label_positions(self):
        EditableGraph._update_node_label_positions(self)
        xmin, xmax = self.ax.get_xlim()
        ymin, ymax = self.ax.get_ylim()

        for node, node_label_artist in self.node_label_artists.items():
            x = node_label_artist._x
            y = node_label_artist._y
            if x > xmin and x < xmax and y > ymin and y < ymax:
                node_label_artist.set_visible(True)
            else:
                node_label_artist.set_visible(False)

    def update_color_node(self):
        for node, node_artist in self.node_artists.items():
            node_artist.set_facecolor(NEUTRAL_COLOR)
            if self.state.humans_node[node]:
                node_artist.set_facecolor(OCCUPIED_COLOR)
            if node == self.state.goal:
                node_artist.set_facecolor(GOAL_COLOR)
            if node == self.state.robot_node:
                node_artist.set_facecolor(ROBOT_COLOR)
        self.fig.canvas.draw_idle()

    def set_axes_limits(self, pad=0.5):
        node_positions = list(self.node_positions.values())
        current_xmin, current_xmax, current_ymin, current_ymax = self.ax.axis()
        xmin = min(min(node_positions, key = lambda t: t[0])[0] - pad, current_xmin)
        xmax = max(max(node_positions, key = lambda t: t[0])[0] + pad, current_xmax) 
        ymin = min(min(node_positions, key = lambda t: t[1])[1] - pad, current_ymin)
        ymax = max(max(node_positions, key = lambda t: t[1])[1] + pad, current_ymax)

        self.ax.axis([xmin, xmax, ymin, ymax])
            
class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        super(MplCanvas, self).__init__(Figure(figsize=(width, height), dpi=dpi))
        self.setParent(parent)
        self.axes = self.figure.add_subplot(111)
        self.axes.patch.set_edgecolor('black')
        self.axes.patch.set_linewidth(1)
        self.mpl_connect('draw_event', self.on_resize)

        self.select_type_node = "none"
        self.graph = None
        self.state = None
    
    def draw(self):
        FigureCanvasQTAgg.draw(self)

        if self.graph is not None:
            self.graph._update_view()
        # self.axes.cla()
        # self.axes.patch.set_edgecolor('black')
        # self.axes.patch.set_linewidth(1)

    def on_resize(self, event):

        print(event)

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

    def init(self, G):
        self.G = G
        self.pos = nx.get_node_attributes(G,'pos')
        self.state = GraphState(None, None, [0 for n in self.G.nodes])
        self.graph = MyGraph(self.state, self.G,
                                    node_layout=self.pos, 
                                    node_size=10,
                                    node_edge_width = 2,
                                    node_labels=True,
                                    node_label_fontdict=dict(size=10),
                                    edge_width=3,
                                    ax=self.canvas.axes)

        self.updated.emit("New environment: " + str(self.G))
        # self.parent().statusBar().showMessage()

    def update(self, state):
        self.state = state
        if self.state is not None:
            self.updated.emit("New state: " + str(self.state))










    def set_graph(self, G):
        self.canvas.axes.cla()
        self.G = G
        self.pos = nx.get_node_attributes(G,'pos')
        self.state = GraphState(None, None, [0 for n in self.G.nodes])
        self.graph = MyGraph(self.state, self.G,
                                    node_layout=self.pos, 
                                    node_size=10,
                                    node_edge_width = 2,
                                    node_labels=True,
                                    node_label_fontdict=dict(size=10),
                                    edge_width=3,
                                    ax=self.canvas.axes)
        self.canvas.axes.patch.set_edgecolor('black')
        self.canvas.axes.patch.set_linewidth(1)

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

    def is_state_valid(self):
        return self.state is not None and self.state.robot_node is not None and self.state.goal is not None
        