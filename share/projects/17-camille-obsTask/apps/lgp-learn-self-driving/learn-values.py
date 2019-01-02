#!/usr/bin/env python

import sys, os
import numpy as np
from joblib import dump, load
from sklearn import svm
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mlxtend.plotting import plot_decision_regions

def find_delimiter_row(data):
  if data.shape[0] == 0:
    return -1

  for i, v in enumerate(data[0]):
    if np.isnan(v):
      return i

  return -1

def plot_values(X, values):
  fig = plt.figure()
  ax = fig.gca(projection='3d')

  for i in range(0,values.shape[1]):
    ax.plot_trisurf(X[:,0], X[:,1], values[:,i], linewidth=0.2, antialiased=True)

  plt.show()


def analyse(dataset_filepath, output_model_filepath, output_image_dir):
  with open(dataset_filepath) as f:
    data = np.loadtxt(f, delimiter=";")

  I = find_delimiter_row(data)

  X = data[:,0:I]
  values = data[:, I+1:]

  plot_values(X, values)

if __name__ == "__main__":
  if len(sys.argv) > 1:
    dataset_filepath = sys.argv[1]
    directory = os.path.dirname(dataset_filepath)
    output_model_filepath = os.path.join(directory, "learned_values.joblib")
    output_image_dir = directory
    analyse(dataset_filepath, output_model_filepath, output_image_dir)


