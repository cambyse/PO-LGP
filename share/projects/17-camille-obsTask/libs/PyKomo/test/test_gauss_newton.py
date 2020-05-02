import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimizers import GaussNewton

SHOW_PLOTS = True

def parabol(x):
    return np.asarray([(x-10.0]), np.asarray([1.0])

def test_affine():
    # minimize (x - 10)^2
    gn = GaussNewton(affine)
    x = gn.run(np.asarray([1.0]))
    npt.assert_almost_equal(x, np.asarray([10.0]), 0.0001)

def dist2d(x):
    # dist from center at (10, 5) in 2d
    dx = x[0, 0] - 10
    dy = x[1, 0] - 5
    gamma = np.sqrt(dx*dx+ dy*dy)
    costheta = dx / gamma
    sintheta = dy / gamma
    return np.asarray([[gamma]]), np.asarray([[costheta, sintheta]])

def test_dist2d():
    # minimize dist from center in 2d
    # 2 input variable, one cost: 2->1
    gn = GaussNewton(dist2d)
    x0 = np.array([[1.0, 1.0]]).T
    x = gn.run(x0)
    npt.assert_almost_equal(x, np.asarray([[10.0, 5.0]]).T, 0.0001)

def vec_function(x):
    # one input two outputs
    f1 = x[0, 0] -10
    f2 = x[0, 0] - 5
    return np.asarray([[f1], [f2]]), np.asarray([[1.0], [1.0]])

def test_vec_function():
    gn = GaussNewton(vec_function)
    x0 = np.array([[1.0]]).T
    x = gn.run(x0)
    npt.assert_almost_equal(x, np.asarray([[7.5]]), 0.0001)

if __name__ == "__main__":
     test_vec_function()