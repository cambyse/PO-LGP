import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimizers import GaussNewton, GaussNewtonFunction

SHOW_PLOTS = True

class Parabol(GaussNewtonFunction):
    def value(self, x):
        return np.asarray([x[0,0]-10.0])

    def gradient(self, x):
        return np.asarray([[1.0]])

def test_Parabol():
    # minimize (x - 10)^2
    f = Parabol()
    gn = GaussNewton(f)
    x = gn.run(np.asarray([[1.0]]))
    npt.assert_almost_equal(x, np.asarray([[10.0]]), 0.0001)

class SquareDistance(GaussNewtonFunction):
    def value(self, x):
    # dist from center at (10, 5) in 2d
        dx = x[0, 0] - 10
        dy = x[1, 0] - 5
        gamma = np.sqrt(dx*dx+ dy*dy)
        return np.asarray([[gamma]])

    def gradient(self, x):
        dist = self.value(x)[0, 0]
        dx = x[0, 0] - 10
        dy = x[1, 0] - 5
        costheta = dx / dist
        sintheta = dy / dist
        return np.asarray([[costheta, sintheta]])

def test_SquareDistance():
    # minimize dist from center in 2d
    # 2 input variable, one cost: 2->1
    f = SquareDistance()
    gn = GaussNewton(f)
    x0 = np.array([[1.0, 1.0]]).T
    x = gn.run(x0)
    npt.assert_almost_equal(x, np.asarray([[10.0, 5.0]]).T, 0.0001)

class Parabol2D(GaussNewtonFunction):
    def value(self, x):
    # one input two outputs
        f1 = x[0, 0] -10
        f2 = x[0, 0] - 5
        return np.asarray([[f1], [f2]])

    def gradient(self, x):
        return np.asarray([[1.0], [1.0]])

def test_Parabol2D():
    f = Parabol2D()
    gn = GaussNewton(f)
    x0 = np.array([[1.0]]).T
    x = gn.run(x0)
    npt.assert_almost_equal(x, np.asarray([[7.5]]), 0.0001)

if __name__ == "__main__":
     test_Parabol2D()