#ifndef DMP_H
#define DMP_H

#include <Core/util.h>
#include <Core/array.h>
#include <Ors/ors.h>


struct DMP {
  DMP(arr &y_ref_, uint nBase_, double dt_);
  ~DMP();

  void trainDMP(double T_=-1., const arr& goal_=NoArr);
  void iterate();
  void plotDMP();
  void printDMP();
  void reset();


  double tau; // Time constant, T = 0.5/tau
  double T; // Motion time [s]
  double dt; // Time step rate [s]
  uint nBase; // Number of basis functions for force
  uint dimY; // Dimension of the DMP
  arr weights; // Weights of the force f = phi(x)'*w
  arr amp; // Amplitude of the DMP

  arr y0; // Start state

  arr goal; // Goal state

  double alphax; // Canonical system constant
  double alphay; // Dynamics constant
  double betay; // Dynamics constant

  arr C; // Centers of basis functions
  arr  H; // Widths of basis functions

  double X; // Canonical system state
  double Xd;

  arr Y; // Dynamical system states
  arr Yd;
  arr Ydd;

  arr y_ref; // Reference trajectory

  // bookkeeping
  arr y_bk;
  arr yd_bk;


};

#endif // DMP_H
