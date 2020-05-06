import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from newton import NewtonFunction
from gauss_newton import SquareCostFunction

class ProjX(NewtonFunction):
    def value(self, x):
        return x[0] #np.asarray([x[0]])

    def gradient(self, x):
        return np.asarray([1.0, 0.0])

    def hessian(self, x):
        return np.zeros((2, 2))

class ProjY(NewtonFunction):
    def value(self, x):
        return x[1] #np.asarray([x[0]])

    def gradient(self, x):
        return np.asarray([0.0, 1.0])

    def hessian(self, x):
        return np.zeros((2, 2))

class SquareDistance(SquareCostFunction):
    def __init__(self, px=10, py=2):
        self.px = px
        self.py = py

    def phi(self, x):
    # dist from center at (10, 5) in 2d
        dx = x[0] - self.px
        dy = x[1] - self.py
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
        return x*x

    def gradient(self, x):
        return 2*x

    def hessian(self, x):
        return 2.0

class Hyperbol(NewtonFunction):
    def value(self, x):
        return x*x*x

    def gradient(self, x):
        return 3*x*x

    def hessian(self, x):
        return 6*x