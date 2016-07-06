#pragma once

#include "taskMap.h"

struct TaskMap_AboveBox : TaskMap {
  int i, j;               ///< which shapes does it refer to?
  TaskMap_AboveBox(int iShape=-1, int jShape=-1);

  TaskMap_AboveBox(const ors::KinematicWorld& G,
                   const char* iShapeName=NULL, const char* jShapeName=NULL);

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 4; }
  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return STRING("TM_AboveBox"<<'_'<<(i<0?"WORLD":G.shapes(i)->name) <<'_' <<(j<0?"WORLD":G.shapes(j)->name)); }
};
