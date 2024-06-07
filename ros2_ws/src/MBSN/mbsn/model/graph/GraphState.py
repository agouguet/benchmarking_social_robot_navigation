from mbsn.model.interfaces.IState import IState

class GraphState(IState):
    def __init__(self, robot_node, goal, humans_node):
        self.robot_node = robot_node
        self.humans_node = humans_node
        self.goal = goal

    def set_robot_node(self, node):
        self.robot_node = node

    def __str__(self):
        return "s" + "(" + str(self.robot_node) + ", " + str(self.goal) + ", " + str([h for h in range(len(self.humans_node)) if self.humans_node[h] != 0]) + ")"

    def __eq__(self, other):
        return other != None and self.robot_node == other.robot_node and tuple(self.humans_node) == tuple(other.humans_node) and self.goal == other.goal

    def __hash__(self):
        return hash((self.robot_node, tuple(self.humans_node), self.goal))

