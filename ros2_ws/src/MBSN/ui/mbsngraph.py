import os, sys, math, random, copy, yaml, cv2
from netgraph import Graph
from netgraph import BaseGraph, EditableGraph, InteractiveGraph
from netgraph._artists import NodeArtist, EdgeArtist
from mbsn.model.graph.GraphState import GraphState
from ui.add_human_node_dialog_box import *

import networkx as nx

NEUTRAL_COLOR = "#a6fbff"
ROBOT_COLOR = "#ffdaa6"
GOAL_COLOR = "#9aff75"
OCCUPIED_COLOR = "#ffa6b3"
ROBOT_PATH = "#b56e09"
OCCUPIED_PATH = "#8a0418"

BASE_SCALE = 1e-2
DEFAULT_COLOR = '#2c404c' # '#677e8c' # '#121f26' # '#23343f' # 'k',


class MBSNGraph(EditableGraph):

    def __init__(self, model, *args, **kwargs):
        self.model = model
        self.state = self.model.state
        self.model.state_updated.connect(self.test)
        G = self.model.G
        pos = nx.get_node_attributes(G,'pos')
        EditableGraph.__init__(self, G,    
                                        node_layout=pos,
                                        node_size=10,
                                        node_edge_width = 2,
                                        node_labels=True,
                                        node_label_fontdict=dict(size=10),
                                        edge_width=3,
                                        *args, **kwargs)
        self.set_axes_limits()

        edge_labels = {x:round(math.dist(self.node_positions[x[0]], self.node_positions[x[1]]), 2) for i,x in enumerate(self.edges)}
        
        self.edge_label_fontdict = self._initialize_edge_label_fontdict(None)
        self.edge_label_position = 0.5
        self.edge_label_rotate = True
        self.edge_label_artists = dict()
        self.draw_edge_labels(edge_labels, self.edge_label_position, self.edge_label_rotate, self.edge_label_fontdict)
        
        self.node_type_to_place = "none"
        # self.emphasizeable_artists = []
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
                self.model.change_robot_node(closest_node)
                # self.model.state = GraphState(closest_node, self.state.goal, self.state.humans_node)
            elif self.node_type_to_place == "goal":
                self.model.change_goal_node(closest_node)
                # self.model.state = GraphState(self.model.state.robot_node, closest_node, self.state.humans_node)
            elif self.node_type_to_place == "human":
                if self.state.humans_node[closest_node]:
                    self.model.state.humans_node[closest_node] = False
                else:
                    dlg = AddHumanNodeDlg(self.model, closest_node)
                    if dlg.exec():
                        start, end = dlg.get_values_of_nodes()
                        self.model.add_human_path(start, end)

                # self.state.humans_node[closest_node] = not self.state.humans_node[closest_node]

            # self.state_changed.emit(True)
            self.update_color_node()
        
        if self._currently_dragging:
            self.update_model_graph()

        super()._on_release(event)
        
    def _move(self, event):
        EditableGraph._move(self, event)
        
    def _add_node(self, event):
        EditableGraph._add_node(self, event)
        self.state.humans_node.append(0)
        self.update_model_graph()

    def _add_edge(self, event):
        EditableGraph._add_edge(self, event)
        self.update_model_graph()
        
    def _update_edge_label_positions(self, edges):
        EditableGraph._update_edge_label_positions(self, edges)

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

    def update(self, state):
        self.state = state
        self.update_color_node()
        self.fig.canvas.draw_idle()

    def test(self):
        self.set_axes_limits()
        self.update_color_node()
        self.fig.canvas.draw_idle()
        # self.update_model_graph()

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
            if self.model.state.humans_node[node]:
                node_artist.set_facecolor(OCCUPIED_COLOR)
            if node == self.model.state.goal:
                node_artist.set_facecolor(GOAL_COLOR)
            if node == self.model.state.robot_node:
                node_artist.set_facecolor(ROBOT_COLOR)

        self.update_path_color()

        self.set_axes_legend()
        self.fig.canvas.draw_idle()
        self.fig.tight_layout()

    def update_path_color(self):
        robot_path = self.model.get_robot_path()
        for node in self.node_artists:
            self.node_artists[node].set_edgecolor(DEFAULT_COLOR)

        for h in self.model._humans:
            for node in h._history:
                self.node_artists[node].set_edgecolor(OCCUPIED_PATH)

        for node in robot_path:
            self.node_artists[node].set_edgecolor(ROBOT_PATH)

        for edge in self.edge_artists:
            self.edge_artists[edge].set_color(DEFAULT_COLOR)

        for h in self.model._humans:
            hp = h._history
            for ii, node_1 in enumerate(hp[:-1]):
                node_2 = hp[ii+1]
                if node_1 != node_2:
                    if (node_1, node_2) in self.edges:
                        edge = (node_1, node_2)
                    else: # the edge is specified in reverse node order
                        edge = (node_2, node_1)
                    self.edge_artists[edge].set_color(OCCUPIED_PATH)

        for ii, node_1 in enumerate(robot_path[:-1]):
            node_2 = robot_path[ii+1]
            if node_1 != node_2:
                if (node_1, node_2) in self.edges:
                    edge = (node_1, node_2)
                else: # the edge is specified in reverse node order
                    edge = (node_2, node_1)
                self.edge_artists[edge].set_color(ROBOT_PATH)

        


    def update_model_graph(self):
        node_positions = self.node_positions
        edges = list(self.edge_artists.keys())
        G = nx.Graph()
        G.add_nodes_from(list(node_positions.keys()))
        for n, p in node_positions.items():
            G.nodes[n]['pos'] = p
        G.add_edges_from(edges)
        self.model.new_graph(G)

    def set_axes_legend(self):
        from matplotlib.lines import Line2D
        legend_elements = [ Line2D([0], [0], marker='o', color='black', label='Node', lw=0,
                                markerfacecolor=NEUTRAL_COLOR, markersize=10),
                            Line2D([0], [0], marker='o', color='black', label='Robot', lw=0,
                                markerfacecolor=ROBOT_COLOR, markersize=10),
                            Line2D([0], [0], marker='o', color='black', label='Human', lw=0,
                                markerfacecolor=OCCUPIED_COLOR, markersize=10),
                            Line2D([0], [0], marker='o', color='black', label='Goal', lw=0,
                                markerfacecolor=GOAL_COLOR, markersize=10),
                            Line2D([0], [0], color=ROBOT_PATH, label='Robot Path', lw=3),
                            Line2D([0], [0], color=OCCUPIED_PATH, label='Human Path', lw=3)]
        self.ax.legend(handles=legend_elements, loc='upper right')

    def set_axes_limits(self, pad=0.5):
        
        node_positions = list(self.node_positions.values())
        if len(node_positions) > 0:
            current_xmin, current_xmax, current_ymin, current_ymax = self.ax.axis()
            xmin = min(min(node_positions, key = lambda t: t[0])[0] - pad, current_xmin)
            xmax = max(max(node_positions, key = lambda t: t[0])[0] + pad, current_xmax) 
            ymin = min(min(node_positions, key = lambda t: t[1])[1] - pad, current_ymin)
            ymax = max(max(node_positions, key = lambda t: t[1])[1] + pad, current_ymax)

            self.ax.axis([xmin, xmax, ymin, ymax])