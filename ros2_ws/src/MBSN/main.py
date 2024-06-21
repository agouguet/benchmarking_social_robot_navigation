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
MINIMAL_DISTANCE_TO_CREATE_VECTOR = 10

def find_graph_file(root_directory):
    graph_files = []

    for file in os.listdir(root_directory):
        if file.endswith(".pickle"):
            graph_files.append(file)
    return graph_files

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
    
    '''
    MatplotlibWidget Main Application

    Inherits from QMainWindow to 

    '''
    def __init__(self):
        
        QMainWindow.__init__(self)

        loadUi("ui/main.ui", self)
        self.setWindowTitle("MBSN GUI")

        self.model = GraphModel()
        self.qfunction = QTable()

        self.setup()

        self.loadGraphButton.clicked.connect(self.load_graph)
        self.saveGraphButton.clicked.connect(self.save)

        self.robotNodeButton.clicked.connect(self.robot_node)
        self.humanNodeButton.clicked.connect(self.human_node)
        self.goalNodeButton.clicked.connect(self.goal_node)

        self.numberOfTimeSpinBox.valueChanged.connect(self.number_of_time_updated)
        self.number_of_time_updated(self.numberOfTimeSpinBox.value())
        self.listHeuristicFunctionComboBox.currentIndexChanged.connect(self.updateHeuristicFunction)

        self.previousButton.clicked.connect(self.previous_state)
        self.nextButton.clicked.connect(self.next)
        self.endButton.clicked.connect(lambda: self.complete_run(False))
        self.multipleTimeButton.clicked.connect(lambda: self.next(True))
        self.multipleTimeCompleteButton.clicked.connect(lambda: self.complete_run(True))
        self.valueIterationButton.clicked.connect(self.value_iteration)

        

        self.model.state_updated.connect(self.state_updated)
        # self.model.state_changed.connect(self.state_on_graph_changed)

        self.MplWidget.set_model(self.model)
        self.debugMplWidget.set_model(self.model)

        # self.addToolBar(NavigationToolbar(self.MplWidget.canvas, self))

        self.history_state = []

        self.fileName = None

        self.policy = None

        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

    def setup(self):
        self.setup_list_graph_combo_box()
        self.setup_list_heuristic_function_combo_box()
        self.updateHeuristicFunction()
        self.button_update()

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

    def setup_list_heuristic_function_combo_box(self):
        for hf in mbsn.solver.heuristicfunction.HEURISTIC_FUNCTIONS:
            self.listHeuristicFunctionComboBox.addItem(hf)
        self.listHeuristicFunctionComboBox.setCurrentText(mbsn.solver.heuristicfunction.DEFAULT_HEURISTIC_FUNCTION)

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
        self.multipleTimeCompleteButton.setText("⏭ " + str(value) + "x")

    def updateHeuristicFunction(self):
        self.heuristic_function = getattr(mbsn.solver.heuristicfunction, self.listHeuristicFunctionComboBox.currentText())

    def previous_state(self):
        self.model.previous_state()

    def current_mbsn_agent(self, reset=True):
        if reset:
            self.qfunction = QTable(default=-1e10)
            MBSNAgentNode.reset_visits()
        state = self.model.state
        G = self.model.G
        mdp = MBSN(G, state.goal, number_of_detected_human=len(self.model._humans),
                        distance_factor=(self.distanceFactorSlider.value()/100), 
                        social_factor=(self.socialFactorSlider.value()/100))
        
        return MBSNAgentMCTS(mdp, self.qfunction, UpperConfidenceBounds(), heuristic_function=self.heuristic_function)

    def next(self, multiple_time=False):
        action_count = {}
        number = self.numberOfTimeSpinBox.value()
        
        # One Time
        def one_iteration(progress_callback):
            agent = self.current_mbsn_agent()
            state = self.model.state
            root_node, _ = agent.mcts(state, timeout=float(self.timeOutSpinBox.value()))
            action, value = root_node.get_value()
            progress_callback.emit(action)
            return root_node

        def update_state(action):
            new_state = self.model.next_state(action)
            self.history_state.append(new_state)
            self.model.state = new_state

        def result(node):
            debug_str = str(node.state) + ":\n"
            for k, v in self.qfunction.qtable.items():
                if k[0] == node.state:
                    debug_str += "      " + str(k[1]) + ": " + str(v) + "\n"
            self.labelDebugOne.setText(debug_str)

            if self.displayGraphCheckBox.isChecked():
                graph_worker = GraphVisualisationWorker(node, parent=self)
                graph_worker.dataSent.connect(self.set_graph_image_on_image_widget)
                graph_worker.start()

        # N Time
        def n_iteration(progress_callback):
            for n in range(number):
                agent = self.current_mbsn_agent()
                state = self.model.state
                root_node, _ = agent.mcts(state, timeout=float(self.timeOutSpinBox.value()))
                action, value = root_node.get_value()
                progress_callback.emit(action)
            return action_count

        def add_action_to_count(action):
            if action not in action_count:
                action_count[action] = 0
            action_count[action] += 1
            self.count_label.setText(str(sum(action_count.values()))+"/"+str(number))

        def display_action_count(action_count):
            print(action_count)

        # Start Thread
        if not multiple_time:
            worker = Worker(one_iteration)
            worker.signals.progress.connect(update_state)
            worker.signals.result.connect(result)
            self.threadpool.start(worker)
        else:
            worker = Worker(n_iteration)
            worker.signals.progress.connect(add_action_to_count)
            worker.signals.result.connect(display_action_count)
            self.threadpool.start(worker)

    def complete_run(self, multiple_time=False):
        robot_paths = []
        number = self.numberOfTimeSpinBox.value()

        # One Run
        def one_run(progress_callback):
            agent = self.current_mbsn_agent()
            state = self.model.state
            while 1:
                root_node, _ = agent.mcts(state, timeout=float(self.timeOutSpinBox.value()))
                if root_node.mdp.is_terminal(root_node.state):
                    break
                action, value = root_node.get_value()
                state = self.model.next_state(action)
                progress_callback.emit(state)

        def update_state(new_state):
            self.history_state.append(new_state)
            self.model.state = new_state

        # N Run
        def n_run(progress_callback):
            original_state = self.model.state
            original_humans = self.model.humans
            for n in range(number):
                agent = self.current_mbsn_agent()
                self.model.reset()
                path = []
                while 1:
                    root_node, _ = agent.mcts(self.model.state, timeout=float(self.timeOutSpinBox.value()))
                    if root_node.mdp.is_terminal(root_node.state):
                        path.append([self.model.state, None])
                        progress_callback.emit(path)
                        break
                    action, value = root_node.get_value()
                    path.append([self.model.state, action])
                    self.model.state = self.model.next_state(action)
                    time.sleep(0.1)
            self.model.reset()
            return robot_paths

        def add_path(path):
            robot_paths.append(path)
            nb = len(robot_paths)
            self.count_label.setText(str(nb)+"/"+str(number))   

        def display_robot_paths(robot_paths):
            edge_count = defaultdict(lambda: 0)

            for path in robot_paths:
                for state, action in path:
                    if action is not None:
                        edge = (action._position, action._node)
                        edge_count[edge]+=1

            G = self.model.G
            for n in G.nodes():
                G.add_edge(n, n)

            for u,v in G.edges():
                G[u][v]['used'] = 1

            for e, v in edge_count.items():
                n1, n2 = e
                G[n1][n2]['used'] = v

            # weights = [G[u][v]['used'] for u,v in G.edges()]
            edge_width={(u,v):weight for u,v,weight in G.edges(data='used')}


            for e in G.edges():
                print(e)

            fig, ax = plt.subplots(figsize=(5,4))
            MBSNGraph(self.model, G=G, 
                                # edge_width=edge_width, 
                                # edge_label_fontdict=dict(size=3), 
                                legend=False, 
                                ax=ax)
            fig.savefig('test.pdf', dpi=plt.gcf().dpi)    


            def robot_path_to_dict():
                _dict = []
                for path in robot_paths:
                    _path_dict = []
                    for state, action in path:
                        if action is not None:
                            print(type(state.toJson()))
                            _path_dict.append([state.toJson(), action.toJson()])
                        else:
                            _path_dict.append([state.toJson(), None])
                    _dict.append(_path_dict)
                return _dict
            
            # nx.draw(G, nx.get_node_attributes(G,'pos'), width=weights)
            # plt.savefig("filename.png")
            agent = self.current_mbsn_agent()

            json_data = {
                # "mdp": agent.mdp,
                "humans": [[h._node] + h._path for h in self.model.humans],
                "heuristic": agent._heuristic_function.__name__ ,
                "paths": robot_path_to_dict(),
            }

            #TODO pickle data (G, mdp, etc...)

            for k,v in json_data.items():
                print(k, type(v), v)

            with open("results/log/%s.pickle" % self.listGraphComboBox.currentText(), "wb") as output_file:
                pickle.dump(json_data, output_file)

            with open("results/log/%s.json" % self.listGraphComboBox.currentText(), 'w') as f:
                json.dump(json_data, f, indent=4)

            for p in robot_paths:
                print(p)

        # Start Thread
        if not multiple_time:
            worker = Worker(one_run)
            worker.signals.progress.connect(update_state)
            self.threadpool.start(worker)
        else:
            worker = Worker(n_run)
            worker.signals.progress.connect(add_path)
            worker.signals.result.connect(display_robot_paths)
            self.threadpool.start(worker)
            
    def value_iteration(self):
        if self.model.is_state_valid():
            state = self.model.state
            G = self.model.G
            if self.policy is None:
                print(len(self.model._humans))
                mdp = MBSN(G, state.goal, number_of_detected_human=len(self.model._humans),
                                distance_factor=(self.distanceFactorSlider.value()/100), 
                                social_factor=(self.socialFactorSlider.value()/100))

                # for state in mdp.get_states():
                #     print(state)

                solver = ValueIteration(mdp, gamma=0.9)
                solver.train()
                self.policy = solver.full_values
            else:
                action_values = self.policy[state]
                action = max(action_values, key=action_values.get)
                print("STATE:", state, "   ACTION:", action)



    def handleDataSent(self, root_node, action):
        print(root_node.state)

        # for (child, _) in root_node.children[action]:
        #     print(child, child.get_value())

        debug_str = str(self.model.state) + ":\n"
        for k, v in self.qfunction.qtable.items():
            if k[0] == self.model.state:
                debug_str += "      " + str(k[1]) + ": " + str(v) + "\n"
        self.labelDebugOne.setText(debug_str)

        new_state = self.model.next_state(action)
        self.history_state.append(new_state)
        self.model.state = new_state


        if self.displayGraphCheckBox.isChecked():
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
        self.button_update()
    
    def button_update(self):
        if self.model.is_state_valid():
            self.nextButton.setEnabled(True)
            self.endButton.setEnabled(True)
            self.multipleTimeButton.setEnabled(True)
            self.multipleTimeCompleteButton.setEnabled(True)
        else:
            self.nextButton.setEnabled(False)
            self.endButton.setEnabled(False)
            self.multipleTimeButton.setEnabled(False)
            self.multipleTimeCompleteButton.setEnabled(False)

        if len(self.model._robot_path_traveled) > 1:
            self.previousButton.setEnabled(True)
        else:
            self.previousButton.setEnabled(False)

    def history_state_updated(self):
        for i in reversed(range(self.stateHistoryVerticalLayout.count())): 
            self.stateHistoryVerticalLayout.itemAt(i).widget().setParent(None)
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