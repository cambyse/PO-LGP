#! /usr/bin/env python2
import numpy as np
import sys
import csv

D = np.array([]).reshape(0, 13)

"""Read data file and imports data as array"""
for arg in sys.argv[1:]: 
    with open(arg) as f:
            for line in f.readlines():
                if line[0] == '#':
                    continue
                D  = np.vstack([D, np.fromstring(line, dtype=np.float, sep=",")])
                #print(D)
            
X, y = np.hsplit(D, [-1])
