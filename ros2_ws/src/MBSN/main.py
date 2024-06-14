# ------------------------------------------------------
# ---------------------- main.py -----------------------
# ------------------------------------------------------
import os, sys, math, random, copy, yaml, cv2, warnings, pickle, numpy as np, networkx as nx
from PyQt5.QtWidgets import*
from PyQt5.uic import loadUi
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)
from shapely.geometry import box, LineString, MultiLineString, Point, MultiPoint, Polygon, MultiPolygon, GeometryCollection

from mbsn.model.graph.MBSN import MBSN
from mbsn.utils.visibility_graph import VisibilityGraph
from mbsn.solver.MCTS import MBSNAgentMCTS, MBSNAgentNode
from mbsn.solver.qtable import QTable
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.utils.graph_visualisation import GraphVisualisation
from mbsn.model.graph.GraphState import GraphState
from mbsn.model.graph.GraphAction import GraphAction
import matplotlib.pyplot as plt
from mbsn.solver.heuristicfunction import *

from ui.mplwidget import *
from ui.graphvizwidget import *

GRAPH_DIRECTORY = "graphs/"
MINIMAL_DISTANCE_TO_CREATE_VECTOR = 10

def find_graph_file(root_directory):
    graph_files = []

    for file in os.listdir(root_directory):
        if file.endswith(".pickle"):
            graph_files.append(file)
    return graph_files

def reset_random_seed():
    seed = random.randrange(sys.maxsize)
    # rng = random.Random(seed)
    print("Seed was:", seed)

class MCTSWorker(QtCore.QThread):
    dataSent = QtCore.pyqtSignal(MBSNAgentNode, GraphAction, int)

    def __init__(self, agent, state, timeout, until_end, multiple_time=None, parent=None):
        super(MCTSWorker, self).__init__(parent)
        self._agent = agent
        self._state = copy.deepcopy(state)
        self._timeout = timeout
        self._until_end = until_end
        self._multiple_time = multiple_time

    def run(self):
        # reset_random_seed()
        if self._multiple_time is None:
            if not self._until_end:
                root_node, num_rollouts = self._agent.mcts(self._state, timeout=self._timeout)
                best_action = root_node.get_value()[0]
                self.dataSent.emit(root_node, best_action, num_rollouts)
            else:
                while 1:
                    root_node, num_rollouts = self._agent.mcts(self._state, timeout=self._timeout)
                    if root_node.mdp.is_terminal(root_node.state):
                        break
                    best_action = root_node.get_value()[0]
                    new_state = copy.deepcopy(root_node.get_outcome_child(best_action).state)
                    self._state = new_state
                    self.dataSent.emit(root_node, best_action, num_rollouts)
        else:
            for i in range(self._multiple_time):
                self._agent.qfunction = QTable(default=-10000)
                MBSNAgentNode.reset_visits()
                root_node, num_rollouts = self._agent.mcts(self._state, timeout=self._timeout)
                best_action = root_node.get_value()[0]
                self.dataSent.emit(root_node, best_action, num_rollouts)

class GraphVisualisationWorker(QtCore.QThread):
    dataSent = QtCore.pyqtSignal(bool)

    def __init__(self, node, parent=None):
        super(GraphVisualisationWorker, self).__init__(parent)
        self.node = node

    def run(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            gv = GraphVisualisation(max_level=100)
            graph = gv.single_agent_mcts_to_graph(self.node, filename="mcts")
            graph.render()
        self.dataSent.emit(True)
                
class VLine(QFrame):
    def __init__(self):
        super(VLine, self).__init__()
        self.setFrameShape(self.VLine|self.Sunken)

class MatplotlibWidget(QMainWindow):
    
    def __init__(self):
        
        QMainWindow.__init__(self)

        loadUi("ui/main.ui", self)
        self.setWindowTitle("MBSN GUI")

        self.setup()

        self.loadGraphButton.clicked.connect(self.load_graph)
        self.saveGraphButton.clicked.connect(self.save)

        self.robotNodeButton.clicked.connect(self.robot_node)
        self.humanNodeButton.clicked.connect(self.human_node)
        self.goalNodeButton.clicked.connect(self.goal_node)

        self.numberOfTimeSpinBox.valueChanged.connect(self.number_of_time_updated)
        self.number_of_time_updated(self.numberOfTimeSpinBox.value())

        self.previousButton.clicked.connect(self.previous_state)
        self.nextButton.clicked.connect(self.next)
        self.endButton.clicked.connect(lambda: self.next(True))
        self.multipleTimeButton.clicked.connect(self.multiple_time)

        self.model = GraphModel()
        self.qfunction = QTable()

        self.model.state_updated.connect(self.state_updated)
        # self.model.state_changed.connect(self.state_on_graph_changed)

        self.MplWidget.set_model(self.model)
        self.debugMplWidget.set_model(self.model)

        # self.addToolBar(NavigationToolbar(self.MplWidget.canvas, self))

        self.history_state = []

        self.fileName = None

    def setup(self):
        self.setup_list_graph_combo_box()

        self.num_rollouts_label = QLabel("")
        self.count_label = QLabel("")
        self.statusBar().reformat()
        self.statusBar().setStyleSheet('border: 0; background-color: #FFF8DC;')
        self.statusBar().setStyleSheet("QStatusBar::item {border: none;}") 
        self.statusBar().addPermanentWidget(VLine())
        self.statusBar().addPermanentWidget(self.num_rollouts_label)
        self.statusBar().addPermanentWidget(VLine())
        self.statusBar().addPermanentWidget(self.count_label)

    def setup_list_graph_combo_box(self):
        graph_files = find_graph_file(GRAPH_DIRECTORY)
        for file in graph_files:
            self.listGraphComboBox.addItem(file.split(".")[0])

    def load_graph(self):
        current_map_combo_box = self.listGraphComboBox.currentText()
        with open(GRAPH_DIRECTORY + current_map_combo_box + ".pickle", 'rb') as file:
            self.G = pickle.load(file)
            self.model.state = GraphState(None, None, [0 for n in self.G.nodes])
            self.model.new_graph(self.G)
            self.reset_state_history()
            self.history_state.append(self.model.state)

    def save_graph(self):
        name = QtGui.QFileDialog.getSaveFileName(self, 'Save File')
        file = open(name,'wb')
        file.write(text)
        pickle.dump(self.MplWidget.get_networkx_graph_from_interactive_graph(), file)
        file.close()

    def save(self):
        self.saveAs()
        if self.fileName is not None:
            self.saveAs()
        else:
            with open(self.fileName, 'w') as f:
                f.write(self.editor.toPlainText())

    def saveAs(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getSaveFileName(self, 
            "Save File", "", "All Files(*);;Text Files(*.txt)", options = options)
        if fileName:
            with open(fileName, 'wb') as f:
                pickle.dump(self.MplWidget.get_networkx_graph_from_interactive_graph(), f)
            self.fileName = fileName
            self.setWindowTitle(str(os.path.basename(fileName)) + " - Notepad Alpha[*]")

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

    def number_of_time_updated(self, value):
        self.multipleTimeButton.setText(str(value) + "x")

    def previous_state(self):
        self.model.previous_state()

    def next(self, until_end=False, multiple_time=False):
        if self.model.is_state_valid():
            state = self.model.state
            G = self.model.G
            mdp = MBSN(G, state.goal, 
                            distance_factor=(self.distanceFactorSlider.value()/100), 
                            social_factor=(self.socialFactorSlider.value()/100))
            
            self.qfunction = QTable(default=-10000)
            print(self.qfunction.qtable["T"])
            # return
            MBSNAgentNode.reset_visits()
            agent = MBSNAgentMCTS(mdp, self.qfunction, UpperConfidenceBounds(), heuristic_function=closest_node_to_goal)

            self._worker = MCTSWorker(agent, state, float(self.timeOutSpinBox.value()), until_end)
            self._worker.dataSent.connect(self.handleDataSent)
            self._worker.start()

    def multiple_time(self):
        number = self.numberOfTimeSpinBox.value()
        action_count = {}

        def add_action_to_count(root_node, action, num_rollouts):
            if action not in action_count:
                action_count[action] = 0
            action_count[action] += 1
            self.count_label.setText(str(sum(action_count.values()))+"/"+str(number))

        if self.model.is_state_valid():
            state = self.model.state
            G = self.model.G
            mdp = MBSN(G, state.goal, 
                            distance_factor=(self.distanceFactorSlider.value()/100), 
                            social_factor=(self.socialFactorSlider.value()/100))
            
            agent = MBSNAgentMCTS(mdp, self.qfunction, UpperConfidenceBounds(), heuristic_function=closest_node_to_goal)

            worker = MCTSWorker(agent, state, float(self.timeOutSpinBox.value()), until_end=False, multiple_time=number)
            worker.dataSent.connect(add_action_to_count)

            def print_action_count():
                print(action_count)
                worker.deleteLater()

            worker.finished.connect(print_action_count)
            worker.start()


    def handleDataSent(self, root_node, action, num_rollouts):
        # print(root_node.visits)

        # for (child, _) in root_node.children[action]:
        #     print(child, child.get_value())

        debug_str = str(self.model.state) + ":\n"
        # for k, v in self.qfunction.qtable.items():
        #     if k[0] == self.model.state:
        #         debug_str += "      " + str(k[1]) + ": " + str(v) + "\n"
        self.labelDebugOne.setText(debug_str)

        new_state = self.model.next_state(action)
        self.history_state.append(new_state)
        self.model.state = new_state
        if num_rollouts is not None:
            self.num_rollouts_label.setText(str(num_rollouts))

        graph_worker = GraphVisualisationWorker(root_node, parent=self)
        graph_worker.dataSent.connect(self.set_graph_image_on_image_widget)
        graph_worker.start()

    def reset_state_history(self):
        self.history_state = []
        if self.model.is_state_valid():
            self.history_state.append(self.model.state)
        self.history_state_updated()

    def state_updated(self, state, num_rollouts=None):
        self.statusBar().showMessage("Current state:" + str(state))
        self.history_state_updated()
        
    def history_state_updated(self):
        for i in reversed(range(self.stateHistoryVerticalLayout.count())): 
            self.stateHistoryVerticalLayout.itemAt(i).widget().setParent(None)
        # for state, humans_path in self.model.history:
        #     self.stateHistoryVerticalLayout.addWidget(QPushButton(str(state), self))
        for state in self.model.history:
            self.stateHistoryVerticalLayout.addWidget(QPushButton(str(state), self))

    def state_on_graph_changed(self, boolean):
        self.reset_state_history()

    def set_graph_image_on_image_widget(self, result):
        if result:
            self.graphVizWidget.setPhoto(QtGui.QPixmap('mcts.png'))

app = QApplication([])
window = MatplotlibWidget()
window.show()
app.exec_()