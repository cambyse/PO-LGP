#!/usr/bin/env python

import sys, os
import numpy as np
from joblib import dump, load
from sklearn import svm
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
from mlxtend.plotting import plot_decision_regions

def plot_data(XY, figure_filepath=None):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  fig, ax = plt.subplots()
  ax.set_title('All Data', size=16)
  ax.scatter(X[:,0], X[:,1], c=Y)

  for i, xy in enumerate(X):
    ax.annotate(int(Y[i]), (xy[0], xy[1]))

  if figure_filepath:
    fig.savefig(figure_filepath)

def plot_boundaries(clf, XY, figure_filepath=None):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  # Plot Decision Region using mlxtend's awesome plotting function
  fig, ax = plt.subplots()
  plot_decision_regions(X=X,
                        y=Y,
                        ax=ax,
                        clf=clf,
                        legend=2)
  ax.set_title('SVM Decision Region Boundary', size=16)

  if figure_filepath:
    fig.savefig(figure_filepath)

def print_centroids(XY):
  last_col = XY.shape[1] - 1
  Y = XY[:, last_col].astype(int)
  number_of_skeletons = np.max(Y)+1

  centroids = np.zeros(shape=(number_of_skeletons, 2))

  for i in range(np.max(Y)+1):
    X_of_i = XY[XY[:,last_col] == i][:, 0:2]
    centroid = np.average(X_of_i, axis=0)
    print("centroid of skeleton {} is: {},{}".format(i, centroid[0], centroid[1]))

def retrieve_classes(XY):
  XZ = XY.copy()

  hashToClass = {}
  for i, hash in enumerate(XY[:, -1]):
    if not hash in hashToClass:
      hashToClass[hash]=len(hashToClass)
    XZ[i, -1] = hashToClass[hash]

  return XZ

def separate_data_set(XY):
  training_XY = []
  test_XY = []

  for i, xy in enumerate(XY):
    if i % 2 == 0:
      training_XY.append(xy)
    else:
      test_XY.append(xy)
  return np.array(training_XY), np.array(test_XY)

def learn_model(XY):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  clf = svm.SVC(gamma='scale') #SVC(gamma='auto') #SVC(gamma='scale') #LinearSVC() #class_weight='balanced'  #, C=1.5 #, kernel='poly', degree=1
  print(clf.fit(X, Y))

  return clf

def evaluate_model(clf,XY):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  Y_pred = clf.predict(X)

  return accuracy_score(Y, Y_pred)

def analyse(dataset_filepath, output_model_filepath, output_image_dir):
  with open(dataset_filepath) as f:
    data = np.loadtxt(f, delimiter=";")

  XY = data[:,0:data.shape[1]]

  # separate classes
  XZ = retrieve_classes(XY)
  plot_data(XZ, figure_filepath=os.path.join(output_image_dir, "all_data.svg"))

  # separate data
  training_XY, test_XY = separate_data_set(XZ)
  print_centroids(XZ)

  # learn and plot model
  clf = learn_model(training_XY)
  plot_boundaries(clf, training_XY, figure_filepath=os.path.join(output_image_dir, "decision_regions.svg"))

  # save and log results
  print("accuracy score (training set):{}".format(evaluate_model(clf, training_XY)))
  print("accuracy score (test set):{}".format(evaluate_model(clf, test_XY)))

  dump(clf, output_model_filepath)

  plt.show()

if __name__ == "__main__":
  if len(sys.argv) > 1:
    dataset_filepath = sys.argv[1]
    directory = os.path.dirname(dataset_filepath)
    output_model_filepath = os.path.join(directory, "learned_regions.joblib")
    output_image_dir = directory
    analyse(dataset_filepath, output_model_filepath, output_image_dir)


