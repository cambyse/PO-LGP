/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#pragma once
#include "taskMap.h"

//===========================================================================

enum TaskMap_DefaultType {
  noTMT=0,    ///< non-initialization
  posTMT,     ///< 3D position of reference
  vecTMT,     ///< 3D vec (orientation)
  quatTMT,    ///< 4D quaterion
  posDiffTMT, ///< the difference of two positions (NOT the relative position)
  vecDiffTMT, ///< the difference of two vectors (NOT the relative position)
  quatDiffTMT,///< the difference of 2 quaternions (NOT the relative quaternion)
  vecAlignTMT,///< 1D vector alignment, can have 2nd reference, param (optional) determins alternative reference world vector
  gazeAtTMT,  ///< 2D orthogonality measure of object relative to camera plane
  pos1DTMT
};
extern const char* TaskMap_DefaultType2String[];

struct TaskMap_Default:TaskMap {
  TaskMap_DefaultType type;
  int i, j;               ///< which shapes does it refer to?
  ors::Vector ivec, jvec; ///< additional position or vector
  intA referenceIds; ///< the shapes it refers to DEPENDENT on time

  TaskMap_Default(TaskMap_DefaultType type,
                 int iShape=-1, const ors::Vector& ivec=NoVector,
                 int jShape=-1, const ors::Vector& jvec=NoVector);

  TaskMap_Default(TaskMap_DefaultType type, const ors::KinematicWorld& G,
                 const char* iShapeName=NULL, const ors::Vector& ivec=NoVector,
                 const char* jShapeName=NULL, const ors::Vector& jvec=NoVector);

  TaskMap_Default(const Graph &parameters, const ors::KinematicWorld& G);
  TaskMap_Default(const Node *parameters, const ors::KinematicWorld& G);

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G);
  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return STRING("Default_"<<TaskMap_DefaultType2String[type]<<'_'<<(i<0?"WORLD":G.shapes(i)->name) <<'_' <<(j<0?"WORLD":G.shapes(j)->name)); }
};

