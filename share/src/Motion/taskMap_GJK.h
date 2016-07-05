#pragma once

#include "taskMaps.h"

struct TaskMap_GJK:TaskMap{
  int i, j;               ///< which shapes does it refer to?
//  ors::Vector vec1, vec2; ///< additional position or vector
  bool exact;

  TaskMap_GJK(const ors::Shape *s1, const ors::Shape *s2, bool exact);
  TaskMap_GJK(const ors::KinematicWorld& W, const char* s1, const char* s2, bool exact);
  TaskMap_GJK(const ors::KinematicWorld& W, const Graph& specs, bool exact);
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& W, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 3; }
  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return STRING("TaskMap_GJK"<<(i<0?"WORLD":G.shapes(i)->name) <<'_' <<(j<0?"WORLD":G.shapes(j)->name)); }
};
