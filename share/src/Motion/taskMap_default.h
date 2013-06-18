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

#ifndef _MT_taskMap_default_h
#define _MT_taskMap_default_h

#include "motion.h"

struct DefaultTaskMap:TaskMap {
  DefaultTaskMapType type;
  int i, j;             ///< which body(-ies) does it refer to?
  ors::Transformation irel, jrel; ///< relative position to the body
  arr params;           ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  virtual void phi(arr& y, arr& J, const ors::Graph& G);
  virtual uint phiDim(const ors::Graph& G);
};

#endif
