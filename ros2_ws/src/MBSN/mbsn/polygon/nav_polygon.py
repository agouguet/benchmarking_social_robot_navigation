import itertools
from matplotlib import pyplot as plt
from shapely import Polygon
from shapely.plotting import plot_line, plot_points, plot_polygon


class NavPolygon():

    id_iter = itertools.count()

    def __init__(self, *args, id=None):
        self.id = next(self.id_iter) if id is None else id
        self.polygon = Polygon(*args)
        self.neighbors = []

        self.color=[1.0, 0.0, 0.0, 1.0]
        self.facecolor="sienna"#[0.6, 0.4, 0.2, 0.5]
        self.edgecolor=[0.0, 0.0, 0.0, 1.0]
        self.zorder = 0


    def is_already_plot(self, displayed=None):
        if displayed is None:
            displayed = {}
        if self.id in displayed:
            return True
        displayed[self.id] = True
        return False

    def plot(self, ax=None, add_points=False, add_id=False, displayed=None):
        if ax is None:
            fig, ax = plt.subplots()
        if not self.is_already_plot(displayed):
            self.plot_polygon(ax, add_points, add_id)

    def plot_polygon(self, ax, add_points=False, add_id=False):
        plot_polygon(self.polygon, ax=ax, add_points=add_points, color=self.color, facecolor=self.facecolor, edgecolor=self.edgecolor, zorder=self.zorder)
        if add_id:
            centroid = self.polygon.centroid
            ax.plot(centroid.x, centroid.y-0.1, 'ko', zorder=100)
            ax.text(centroid.x, centroid.y-0.1, str(self.id), fontsize=7, ha='center', va='center', color='white', bbox=dict(facecolor='black', alpha=0.5), zorder=101)

    def add_neighbor(self, neighbor):
        if neighbor not in self.neighbors:
            self.neighbors.append(neighbor)

    def intersects(self, geometry):
        if isinstance(geometry, NavPolygon):
            return self.polygon.intersects(geometry.polygon)
        return self.polygon.intersects(geometry)

    def __str__(self) -> str:
        s = self.polygon.wkt.split(" ", 1)
        return s[0] + " " + str(self.id) + " " + s[1]
    
    def __repr__(self):
        return self.__str__()

    def __getattr__(self, attr):
        # Delegate attribute access to the underlying polygon object
        return getattr(self.polygon, attr)