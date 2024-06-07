from mbsn.model.interfaces.IAction import IAction

class GraphAction(IAction):
    def __init__(self, position, node):
        IAction.__init__(self, str(position) + ", " + str(node))
        self._node = node
        self._position = position