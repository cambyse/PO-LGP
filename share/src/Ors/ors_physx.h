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

/// @file
/// @ingroup group_ors

#ifndef MT_ors_physx_h
#define MT_ors_physx_h
#include "ors.h"

namespace physx {
  class PxMaterial;
}

/**
 * @defgroup ors_interface_physx Interface to PhysX
 * @ingroup ors_interfaces
 * @{
 */
struct PhysXInterface {
  struct sPhysXInterface *s;
  
  PhysXInterface();
  ~PhysXInterface();
  
  void create(ors::Graph& G);
  void step(double tau=1./60.);
  void glDraw();
  
  void pushState();
  void pullState();

  bool isCreated();
  void ShutdownPhysX();
};

void glPhysXInterface(void *classP);


void bindOrsToPhysX(ors::Graph& graph, OpenGL& gl, PhysXInterface& physx);

#endif
/// @}
