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

#pragma once

#include "ors.h"

//===========================================================================
/**
 * @defgroup ors_interface_SWIFT SWIFT Interface.
 * @{
 */

class SWIFT_Scene;

/// contains all information necessary to communicate with swift
struct SwiftInterface {
  SWIFT_Scene *scene;
  bool isOpen;
  intA INDEXswift2shape, INDEXshape2swift;
  double cutoff;
  SwiftInterface() { scene=NULL; cutoff=.1; isOpen=false; }
  ~SwiftInterface();
  SwiftInterface* newClone(const ors::Graph& G) const;

  void setCutoff(double _cutoff){ cutoff=_cutoff; }
  void init(const ors::Graph& ors, double _cutoff=.1);
  void reinitShape(const ors::Graph& ors, const ors::Shape *s);
  void close();
  void deactivate(ors::Shape *s1, ors::Shape *s2);
  void deactivate(const MT::Array<ors::Shape*>& shapes);
  void deactivate(const MT::Array<ors::Body*>& bodies);
  void initActivations(const ors::Graph& ors, uint parentLevelsToDeactivate=3);
  void computeProxies(ors::Graph& ors, bool dumpReport=false);
};
/** @} */
