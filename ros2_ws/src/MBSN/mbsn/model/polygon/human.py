import itertools

from matplotlib import pyplot as plt
import numpy as np
from shapely import Point

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

    @property
    def orientation(self):
        return self._orientation
    
    @orientation.setter
    def orientation(self, orientation):
        self._orientation = orientation

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