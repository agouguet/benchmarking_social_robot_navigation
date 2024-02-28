from graph_nav.model.State import State

class GraphState(State):
    def __init__(self, robot_node, human_node):
        self.robot_node = robot_node
        self.human_node = human_node
        self._name = "(" + str(self.robot_node) + ", " + str(self.human_node) + ")"

    def __eq__(self, other):
        return self._name == other._name and self.robot_node == other.robot_node and self.human_node == other.human_node

    def __hash__(self):
        return hash((self._name, self.robot_node, self.human_node))