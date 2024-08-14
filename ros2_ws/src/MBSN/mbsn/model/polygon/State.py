import json


class State():
    def __init__(self, robot, humans):
        self.robot = robot
        self.humans = humans #TODO Make changement Occupied -> Human


    # @property
    # def robot(self, robot_state):
    #     self.robot = robot_state

    # @property
    # def robot(self):
    #     return self.robot

    def __str__(self):
        return "s" + "(" + str(self.robot) + ", " + str(self.humans) + ")"
    
    def __repr__(self) -> str:
        return self.__str__()

    def __eq__(self, other):
        return other != None and self.robot == other.robot and self.humans == other.humans

    def __hash__(self):
        return hash((self.robot, tuple(self.humans)))

    def toJson(self):
        _dict ={
            "robot": self.robot,
            "humans": self.humans,
        }
        return _dict

