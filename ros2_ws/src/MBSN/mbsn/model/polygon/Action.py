import json

class Action():
    def __init__(self, position, node):
        self._node = node
        self._position = position
        self._name = str(position) + ", " + str(node)

    def __str__(self):
        return "a(" + str(self._name) + ")"

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return self._name == other._name

    def __hash__(self):
        return hash(self._name)

    def toJson(self):
        _dict ={
            "from": self._position,
            "to": self._node,
        }
        return _dict