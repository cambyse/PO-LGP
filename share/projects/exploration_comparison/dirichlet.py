"""
Dirichlet distribution and helper functions.
"""

from __future__ import division
import operator
import random

import numpy as np
import scipy as sp
import scipy.special

import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D


def pdf(loc, alphas):
    """Evaluate the pdf of the dirichlet distribution with parameters alphas at
    loc.

    """
    assert len(loc) == len(alphas)
    assert sum(loc) == 1

    prod = reduce(operator.mul,
                  [loc[i] ** (alphas[i] - 1) for i in range(len(loc))])
    normalizer = reduce(operator.mul, sp.special.gamma(alphas))

    return prod / normalizer


def get_dom(dim, n=10000):
    """Return n samples with the given dim for the dirichlet distribution.

    Rejection sampling is used.

    """
    dom = []
    while True:
        samples = [random.random() for i in range(dim - 1)]
        sum_ = sum(samples)
        if sum_ > 1.:
            continue
        samples.append(1 - sum_)
        dom.append(samples)

        if len(dom) >= n:
            break

    return np.array(dom)


def plot(alphas, dom=None, data_points=10000):
    """Create a plot for the given alphas.

    Automatically create the dom.

    """
    if dom:
        X = dom
    else:
        X = get_dom(len(alphas))

    xs = X[:, 0]
    ys = X[:, 1]
    zs = np.array([pdf(X[i, :], alphas) for i in range(X.shape[0])])

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, ".", color=zs)
