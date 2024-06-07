from mbsn.model.graph.MBSN import MBSN
import pickle
from mbsn.utils.visibility_graph import VisibilityGraph
from mbsn.solver.MCTS import MBSNAgentMCTS, MBSNAgentNode
from mbsn.solver.qtable import QTable
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.utils.graph_visualisation import GraphVisualisation
from mbsn.model.graph.GraphState import GraphState
from mbsn.model.graph.GraphAction import GraphAction
import matplotlib.pyplot as plt

# import networkx as nx

# scenario = "frontal"


# G = pickle.load(open("../unity_sim/graphs/" + scenario + ".pickle", 'rb'))

# robot_node = 0
# goal = 12

# occupied_node = [0 for n in G.nodes]

# humans_node = [5]

# for i in humans_node:
#     occupied_node[i] = 1

# # G, robot_node, goal, occupied_node = example_of_3_nodes()


# mdp = MBSN(G, goal)
# qfunction = QTable()
# root_node = MBSNAgentMCTS(mdp, qfunction, UpperConfidenceBounds()).mcts(GraphState(robot_node, goal, occupied_node), timeout=0.2)



# # print(root_node.get_visits(), MBSNAgentNode.visits)

# # print(qfunction.get_q_value(GraphState(robot_node, goal, occupied_node), 2))
# # print(qfunction.get_q_value(GraphState(robot_node, goal, occupied_node), 1))

# # print(mdp.get_next_states(GraphState(robot_node, goal, occupied_node), GraphAction(2)))

# # print("----------------------")

# # print(qfunction.save("test"))
# # print(qfunction.get_q_value(GraphState(robot_node, goal, [0, 0, 0, 0, 0]), 0))




# # print(qfunction.qtable[0])
# # print(qfunction.qtable[1])
# for k, v in qfunction.qtable.items():
#     print(k, v)


# print(root_node.get_value())

# print(MBSNAgentNode.visits)

# gv = GraphVisualisation(max_level=5)
# graph = gv.single_agent_mcts_to_graph(root_node, filename="mcts")
# graph.view()


# pos=nx.get_node_attributes(G,'pos')
# color_map = ["blue" if i == 0 else "red" for i in occupied_node]
# nx.draw(G,pos, node_color=color_map, with_labels=True)
# plt.show()




# ------------------------------------------------------
# ---------------------- main.py -----------------------
# ------------------------------------------------------
import os, sys, math, random, copy, yaml, cv2
from PyQt5.QtWidgets import*
from PyQt5.uic import loadUi
from PyQt5.QtCore import QObject, QThread, pyqtSignal

from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)
from shapely.geometry import box, LineString, MultiLineString, Point, MultiPoint, Polygon, MultiPolygon, GeometryCollection

import numpy as np
import random
import pickle
import networkx as nx

GRAPH_DIRECTORY = "graphs/"
MINIMAL_DISTANCE_TO_CREATE_VECTOR = 10

def find_graph_file(root_directory):
    graph_files = []

    for file in os.listdir(root_directory):
        if file.endswith(".pickle"):
            graph_files.append(file)
    return graph_files


class MatplotlibWidget(QMainWindow):
    
    def __init__(self):
        
        QMainWindow.__init__(self)

        loadUi("ui/main.ui", self)

        self.setWindowTitle("PyQt5 & Matplotlib Example GUI")

        self.setup()

        self.loadGraphButton.clicked.connect(self.load_graph)
        self.robotNodeButton.clicked.connect(self.robot_node)
        self.humanNodeButton.clicked.connect(self.human_node)
        self.goalNodeButton.clicked.connect(self.goal_node)
        self.nextButton.clicked.connect(self.next)

        self.MplWidget.updated.connect(self.state_updated)

        # self.addToolBar(NavigationToolbar(self.MplWidget.canvas, self))

    def setup(self):
        self.setup_list_graph_combo_box()

    def setup_list_graph_combo_box(self):
        graph_files = find_graph_file(GRAPH_DIRECTORY)
        for file in graph_files:
            self.listGraphComboBox.addItem(file.split(".")[0])

    def load_graph(self):
        current_map_combo_box = self.listGraphComboBox.currentText()
        with open(GRAPH_DIRECTORY + current_map_combo_box + ".pickle", 'rb') as file:
            self.G = pickle.load(file)
            self.MplWidget.init(self.G)

    def robot_node(self):
        self.humanNodeButton.setChecked(False)
        self.goalNodeButton.setChecked(False)
        if self.robotNodeButton.isChecked():
            self.MplWidget.set_select_type_node("robot")
        else:
            self.MplWidget.set_select_type_node("none")

    def human_node(self):
        self.robotNodeButton.setChecked(False)
        self.goalNodeButton.setChecked(False)
        if self.humanNodeButton.isChecked():
            self.MplWidget.set_select_type_node("human")
        else:
            self.MplWidget.set_select_type_node("none")
        
    def goal_node(self):
        self.robotNodeButton.setChecked(False)
        self.humanNodeButton.setChecked(False)
        if self.goalNodeButton.isChecked():
            self.MplWidget.set_select_type_node("goal")
        else:
            self.MplWidget.set_select_type_node("none")

    def next(self):
        if self.MplWidget.is_state_valid():
            state = self.MplWidget.state
            G = self.MplWidget.get_networkx_graph_from_interactive_graph()
            print(state, G)

            mdp = MBSN(G, state.goal)
            qfunction = QTable()

            path = [state.robot_node]

            while state.robot_node != state.goal:
                root_node = MBSNAgentMCTS(mdp, qfunction, UpperConfidenceBounds()).mcts(state, timeout=float(self.timeOutSpinBox.value()))

                # for k, v in qfunction.qtable.items():
                #     print(k, v)

                pos = nx.get_node_attributes(G,'pos')
                print(root_node.get_value(), pos[root_node.get_value()[0]._node])
                state.robot_node = root_node.get_value()[0]._node
                path.append(state.robot_node)
            
            print("PATH = ", path)
            plot_instance = self.MplWidget.graph
            for node in path:
                plot_instance.node_artists[node].radius = plot_instance.node_artists[node].radius * 1.2
                plot_instance.node_artists[node].set_color('orange')

            for ii, node_1 in enumerate(path[:-1]):
                node_2 = path[ii+1]
                if (node_1, node_2) in plot_instance.edges:
                    edge = (node_1, node_2)
                else: # the edge is specified in reverse node order
                    edge = (node_2, node_1)
                plot_instance.edge_artists[edge].update_width(0.5 * 1e-2)
                plot_instance.edge_artists[edge].set_color('red')
                plot_instance.edge_artists[edge].set_alpha(1.0)

    def state_updated(self, msg):
        self.statusBar().showMessage(msg)

app = QApplication([])
window = MatplotlibWidget()
window.show()
app.exec_()