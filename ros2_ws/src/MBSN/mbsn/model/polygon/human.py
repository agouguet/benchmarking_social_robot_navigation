import itertools
import math

from matplotlib import pyplot as plt
import numpy as np
from shapely import Point
from geometry_msgs.msg import Point as ROSMsgPoint, Quaternion
from mbsn.utils.util import euler_from_quaternion# type: ignore

class Human():

    id_iter = itertools.count()

    def __init__(self, position, orientation=0.0, id=None):
        self.position = position
        self.orientation = orientation
        self.id = next(self.id_iter) if id is None else id

    @property
    def position(self):
        return self._position
    
    @position.setter
    def position(self, position):
        if isinstance(position, tuple):
            self._position = Point(position[0], position[1])
        elif isinstance(position, Point):
            self._position = position
        elif isinstance(position, ROSMsgPoint):
            self._position = Point(position.x, position.y)


    @property
    def orientation(self):
        return self._orientation
    
    @orientation.setter
    def orientation(self, orientation):
        if isinstance(orientation, int) or isinstance(orientation, float):
            self._orientation = orientation
        elif isinstance(orientation, Quaternion):
            r, p, y = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            self._orientation = math.degrees(y)

    def move(self, x, y):
        self._position = Point(self.position.x + x, self.position.y + y)

    def plot(self, ax):
        circle = plt.Circle((self.position.x, self.position.y), radius=0.2, color="blue")
        ax.add_patch(circle)
        label = ax.annotate("H", xy=(self.position.x, self.position.y), fontsize=10, ha="center", color="white", verticalalignment="center", horizontalalignment="center")

        # Convertir l'angle de degrés en radians
        angle_rad = np.radians(self.orientation)
        
        # Déterminer les composants x et y de la direction de la flèche
        arrow_length = 0.15  # Longueur de la flèche
        dx = arrow_length * np.cos(angle_rad)
        dy = arrow_length * np.sin(angle_rad)
        arrow = plt.Arrow(self.position.x, self.position.y, dx, dy, width=0.05, edgecolor='red', facecolor='red')
        ax.add_patch(arrow)

    def __eq__(self, other):
        return other != None and self.position == other.position and self.orientation == other.orientation
    
    def __hash__(self):
        return hash((self.id, self.position, self.orientation))

    def __str__(self):
        return  "((" + str(self.position.x) + ", " + str(self.position.y) + "), " + str(self.orientation) + ")"
    
    def __repr__(self) -> str:
        return self.__str__()
    
