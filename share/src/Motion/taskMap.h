#pragma once
#include <Ors/ors.h>

/// defines only a map (task space), not yet the costs in this space
struct TaskMap {
  uint order;       ///< 0=position, 1=vel, etc
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1) = 0; ///< this needs to be overloaded
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1); ///< if not overloaded this computes the generic pos/vel/acc depending on order
  virtual uint dim_phi(const ors::KinematicWorld& G) = 0; //the dimensionality of $y$
  virtual uint dim_phi(const WorldL& G, int t){ return dim_phi(*G.last()); }

  VectorFunction vf(ors::KinematicWorld& G){
    return [this, &G](arr& y, arr& J, const arr& x) -> void {
      G.setJointState(x);
      phi(y, J, G, -1);
    };
  }

  TaskMap():order(0) {}
  virtual ~TaskMap() {}

  static TaskMap *newTaskMap(const Node* specs, const ors::KinematicWorld& world); ///< creates a task map based on specs
};
