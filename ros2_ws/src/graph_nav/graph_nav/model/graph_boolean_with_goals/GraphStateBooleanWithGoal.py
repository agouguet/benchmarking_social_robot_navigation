from graph_nav.model.State import State

class GraphStateBooleanWithGoal(State):
    def __init__(self, robot_node, humans_node, goal):
        self.robot_node = robot_node
        self.humans_node = humans_node
        self.goal = goal
        self._name = "(" + str(self.robot_node) + ", " + str(self.humans_node) + ", " + str(self.goal) + ")"

    def __eq__(self, other):
        return self._name == other._name and self.robot_node == other.robot_node and tuple(self.humans_node) == tuple(other.humans_node) and self.goal == other.goal

    def __hash__(self):
        return hash((self._name, self.robot_node, tuple(self.humans_node), self.goal))