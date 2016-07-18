#! /usr/bin/env python2
import numpy as np
import sys
import csv
from sklearn import linear_model
from sklearn.neighbors import NearestNeighbors 

D = np.array([]).reshape(0, 15)

"""Read data file and imports data as array"""
for arg in sys.argv[1:]: 
    with open(arg) as f:
            for line in f.readlines():
                if line[0] == '#':
                    continue
                D  = np.vstack([D, np.fromstring(line, dtype=np.float, sep=",")])
                #print(D)

def discard_far_points(x, D, eps):
    D_close = np.array([]).reshape(0, 15)
    
    for i in range(D.shape[0]):
        if np.linalg.norm(D[i, :-3] - x) < eps:
            D_close = np.vstack([D_close, D[i,:]])

    return D_close

def kNN(x, D, k):
    nbrs = NearestNeighbors(n_neighbors=k, algorithm='ball_tree').fit(D[:,:-3])
    distances, indices = nbrs.kneighbors(x)
    return D[indices][0]


def estimate_gradient(x, D, k): 
    k = min(D.shape[0], k)

    X, pos, y = np.hsplit(kNN(x, D, k), [-3, -1])
    rreg = linear_model.LinearRegression()
    print(X)
    rreg.fit (X, y)

    return rreg.coef_
