import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from newton import NewtonFunction
from gauss_newton import SquareCostFunction
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
    def __init__(self):
        self.aula_runs = []
        self.x = []
        self.y = []
        self.z = []

        self.aula_start_x =[]
        self.aula_start_y = []
        self.aula_start_z = []

        self.last_x = None

    def new_aula_run(self, x):
        self.aula_start_x.append(x[0])
        self.aula_start_y.append(x[1])
        self.aula_start_z.append(x[2])

        def c(x):
            return copy.deepcopy(x)
        self.aula_runs.append((c(self.x), c(self.y), c(self.z)))
        self.x.clear()
        self.y.clear()
        self.z.clear()

        self.add(x)

    def new_newton_run(self, x):
        self.add(x)

    def add(self, x):
        self.x.append(x[0])
        self.y.append(x[1])
        self.z.append(x[2])

        self.last_x = x

    def close(self):
        self.new_aula_run(self.last_x)

    def plot(self):
        self.close() # flush
        from mpl_toolkits.mplot3d import Axes3D

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        #ax.set_aspect(aspect='equal')

        colors = ['r', 'g', 'b', 'gray', 'yellow']

        for i, (x, y, z) in enumerate(self.aula_runs):
            ax.plot(x, y, z)
            ax.scatter(x, y, z)
        ax.plot(self.aula_start_x, self.aula_start_y, self.aula_start_z, linewidth=6)
        plt.show()

class ProjX(NewtonFunction):
    def value(self, x):
        return x[0] #np.asarray([x[0]])

    def gradient(self, x):
        g = np.zeros(x.shape)
        g[0] = 1.0
        return g

    def hessian(self, x):
        return np.zeros((x.shape[0], x.shape[0]))

class ProjY(NewtonFunction):
    def value(self, x):
        return x[1] #np.asarray([x[0]])

    def gradient(self, x):
        return np.asarray([0.0, 1.0])

    def hessian(self, x):
        return np.zeros((2, 2))

class SquareDistance(SquareCostFunction):
    def __init__(self, cx=10, cy=2):
        self.cx = cx
        self.cy = cy

    def phi(self, x):
    # dist from center at (10, 5) in 2d
        dx = x[0] - self.cx
        dy = x[1] - self.cy
        return np.asarray([dx, dy])

    def gradientPhi(self, x):
        return np.asarray([[1.0, 0.0], [0.0, 1.0]])


class Parabol(SquareCostFunction):
    def phi(self, x):
        return np.asarray([x[0]-10.0])

    def gradientPhi(self, x):
        return np.asarray([[1.0]])

class Parabol2D(SquareCostFunction):
    def phi(self, x):
    # one input two outputs
        f1 = x[0] -10
        f2 = x[0] - 5
        return np.asarray([f1, f2])

    def gradientPhi(self, x):
        return np.asarray([[1.0], [1.0]])


class PureParabol(NewtonFunction):
    def value(self, x):
        return np.dot(x.T, x)

    def gradient(self, x):
        return np.array([2*x[0]])

    def hessian(self, x):
        return np.array([[2.0]])

class Hyperbol(NewtonFunction):
    def value(self, x):
        return x*x*x

    def gradient(self, x):
        return 3*x*x

    def hessian(self, x):
        return 6*x


class SquareDistance3D(SquareCostFunction):
    def __init__(self, cx=10, cy=2, cz=1):
        self.cx = cx
        self.cy = cy
        self.cz = cz

    def phi(self, x):
        dx = x[0] - self.cx
        dy = x[1] - self.cy
        dz = x[2] - self.cz
        return np.asarray([dx, dy, dz])

    def gradientPhi(self, x):
        return np.asarray([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

class SquareDistance3DDecomp0(SquareCostFunction):
    def __init__(self, cx=10, cy=2):
        self.cx = cx
        self.cy = cy
        self.x_factor = np.sqrt(0.5)

    def phi(self, x):
        dx = x[0] - self.cx
        dy = x[1] - self.cy
        return np.asarray([self.x_factor * dx, dy, 0])

    def gradientPhi(self, x):
        return np.asarray([[self.x_factor, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])

class SquareDistance3DDecomp1(SquareCostFunction):
    def __init__(self, cx=10, cz=1):
        self.cx = cx
        self.cz = cz
        self.x_factor = np.sqrt(0.5)

    def phi(self, x):
        dx = x[0] - self.cx
        dz = x[2] - self.cz
        return np.asarray([self.x_factor * dx, 0, dz])

    def gradientPhi(self, x):
        return np.asarray([[self.x_factor, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

class SphereConstraint3D(SquareCostFunction):
    def __init__(self, cx, cy, cz, radius):
        self.sqdist = SquareDistance3D(cx, cy, cz)
        self.sqradius = radius * radius

    def value(self, x):
        return self.sqdist.value(x) - self.sqradius

    def gradient(self, x):
        return self.sqdist.gradient(x)

    def hessian(self, x):
        return self.sqdist.hessian(x)