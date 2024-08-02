import json

class State():
    def __init__(self, robot, occupied):
        self.robot = robot
        self.occupied = occupied

    # @property
    # def robot(self, robot_state):
    #     self.robot = robot_state

    # @property
    # def robot(self):
    #     return self.robot

    def __str__(self):
        return "s" + "(" + str(self.robot) + ", " + str([id for id, o in self.occupied.items() if o == 1]) + ")"
    
    def __repr__(self) -> str:
        return self.__str__()

    def __eq__(self, other):
        return other != None and self.robot == other.robot and self.occupied == other.occupied

    def __hash__(self):
        return hash((self.robot, tuple(self.occupied)))

    def toJson(self):
        _dict ={
            "robot": self.robot,
            "occupied": self.occupied,
        }
        return _dict

