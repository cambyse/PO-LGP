/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#ifndef _MT_taskMap_constrained_h
#define _MT_taskMap_constrained_h

#include "motion.h"

struct CollisionConstraint:public TaskMap {
  double margin;

  CollisionConstraint():margin(.1){ constraint=true; }

  virtual void phi(arr& y, arr& J, const ors::Graph& G);
  virtual uint dim_phi(const ors::Graph& G){ return 1; }
};

struct PlaneConstraint:public TaskMap {
  int i;       ///< which shapes does it refer to?
  arr planeParams;  ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  PlaneConstraint(const ors::Graph& G, const char* iShapeName, const arr& _planeParams):
    i(G.getShapeByName(iShapeName)->index), planeParams(_planeParams){ constraint=true; }

  virtual void phi(arr& y, arr& J, const ors::Graph& G);
  virtual uint dim_phi(const ors::Graph& G){ return 1; }
};

#endif
