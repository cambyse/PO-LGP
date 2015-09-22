#pragma once

#include <Core/array.h>
#include <Algo/eigenValues.h>
#include <Gui/mesh.h>
#include "dataNeighbored.h"

struct MinEigModel{
  DataNeighbored& data;
  arr weights;

  //statistics
  arr bias_xx;
  double stat_n;
  arr stat_x, stat_xx;

  //eigen value/vector
  arr mean;
  ExtremeEigenValues eig;

  //used points
  uintA pts;      ///< set of points that this model models (has non-zero weights)
  uintA fringe;   ///< the fringe of model points (subset of pts)
  boolA included; ///< binary indicator encoding of pts (is equivalent to pts)

  //cvx hull
  ors::Mesh convexHull;
  double density;

  //label
  int label;


  MinEigModel(DataNeighbored& data) : data(data), weights(zeros(data.n())), label(0) {}

  void setPoints(const uintA& points); ///< set the model points (weights initialized to one)
  void setWeightsToOne();    ///< set all weights in pts to 1
  void setWeightsToZero();   ///< set all weights (ans stats) to zero
  void calc(bool update);    ///< compute the model (calc the eigenvalue/vector) incremental/update or exact
  void expand(uint steps=1); ///< add all neighbors of the fringe
  void reweightWithError(uintA& pts, double margin=0.01);
  void computeConvexHull();
  void calcDensity();
  void colorPixelsWithWeights(arr& cols);
  void glDraw();
  void report(ostream& os=std::cout, bool mini=false);

private:
  void addStatistics(const uintA& points, bool minus=false);
};
