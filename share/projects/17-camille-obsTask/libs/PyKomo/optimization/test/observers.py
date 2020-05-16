import sys
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
import matplotlib.pyplot as plt
import copy

class Plotter2D:
    def __init__(self):
        self.x = []
        self.y = []

    def add(self, x):
        self.x.append(x[0])
        self.y.append(x[1])

    def plot(self):
        _, ax = plt.subplots()
        ax.set_aspect(aspect='equal')
        ax.scatter(self.x, self.y)
        plt.show()

class Plotter3D:
    def __init__(self, title):
        self.aula_runs = []
        self.x = []
        self.y = []
        self.z = []
        self.title = title

        self.aula_start_x =[]
        self.aula_start_y = []
        self.aula_start_z = []

        self.last_x = None

        self.n_evals = 0

    def on_aula_end(self, x):
        self.add_point(x)

        def c(x):
            return copy.deepcopy(x)
        self.aula_runs.append((c(self.x), c(self.y), c(self.z)))
        self.x.clear()
        self.y.clear()
        self.z.clear()

    def on_aula_start(self, x):
        self.aula_start_x.append(x[0])
        self.aula_start_y.append(x[1])
        self.aula_start_z.append(x[2])

        self.add_point(x)

    def on_newton_start(self, x):
        self.add_point(x)

    def on_newton_end(self, x):
        self.add_point(x)

    def on_newton_step(self, x): # x is value at start on newton step
        self.n_evals += 1

    def add_point(self, x):
        self.x.append(x[0])
        self.y.append(x[1])
        self.z.append(x[2])

        self.last_x = x

    def report(self, plot=False):
        print("Number of evaluations:{}".format(self.n_evals))

        if plot:
            from mpl_toolkits.mplot3d import Axes3D

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_title(self.title)
            #ax.set_aspect(aspect='equal')

            colors = ['r', 'g', 'b', 'gray', 'yellow']

            for i, (x, y, z) in enumerate(self.aula_runs):
                ax.plot(x, y, z)
                ax.scatter(x, y, z)
            ax.plot(self.aula_start_x, self.aula_start_y, self.aula_start_z, linewidth=6)
            plt.show()
