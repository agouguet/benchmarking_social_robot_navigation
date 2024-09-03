

from abc import abstractmethod


class TrajectoryPredictionModel():

    def __init__(self) -> None:
        pass

    @abstractmethod
    def predict(self, state, action):
        pass