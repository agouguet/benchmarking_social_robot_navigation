from mbsn.model.interfaces.IAction import IAction

class GraphAction(IAction):
    def __init__(self, node):
        IAction.__init__(self, str(node))
        self._node = node