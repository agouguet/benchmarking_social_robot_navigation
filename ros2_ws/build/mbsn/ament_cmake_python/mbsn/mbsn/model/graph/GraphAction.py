from mbsn.model.interfaces.IAction import IAction
import json

class GraphAction(IAction):
    def __init__(self, position, node):
        IAction.__init__(self, str(position) + ", " + str(node))
        self._node = node
        self._position = position

    def toJson(self):
        _dict ={
            "from": self._position,
            "to": self._node,
        }
        return _dict