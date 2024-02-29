from abc import ABC, abstractmethod

class IState(ABC):
    def __init__(self, name):
        self._name = name

    def __str__(self):
        return "s" + str(self._name)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return self._name == other._name

    def __hash__(self):
        return hash(self._name)