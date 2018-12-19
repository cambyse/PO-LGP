#!/usr/bin/env python

import sys
import numpy as np
from sklearn import svm
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
from mlxtend.plotting import plot_decision_regions

def plot_data(XY):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  fig, ax = plt.subplots()
  ax.set_title('All Data', size=16)
  ax.scatter(X[:,0], X[:,1], c=Y)

  for i, xy in enumerate(X):
    ax.annotate(int(Y[i]), (xy[0], xy[1]))


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

  # Plot Decision Region using mlxtend's awesome plotting function
  fig, ax = plt.subplots()
  plot_decision_regions(X=X,
                        y=Y,
                        ax=ax,
                        clf=clf,
                        legend=2)
  ax.set_title('SVM Decision Region Boundary', size=16)

  return clf

def evaluate_model(clf,XY):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  Y_pred = clf.predict(X)

  return accuracy_score(Y, Y_pred)

def analyse(filename):
  with open(filename) as f:
    data = np.loadtxt(f, delimiter=";")

  XY = data[:,0:data.shape[1]]

  plot_data(XY)

  training_XY, test_XY = separate_data_set(XY)

  clf = learn_model(training_XY)

  print("accuracy score (training set):{}".format(evaluate_model(clf, training_XY)))
  print("accuracy score (test set):{}".format(evaluate_model(clf, test_XY)))

  plt.show()


if __name__ == "__main__":
  if len(sys.argv) > 1:
    analyse(sys.argv[1])


