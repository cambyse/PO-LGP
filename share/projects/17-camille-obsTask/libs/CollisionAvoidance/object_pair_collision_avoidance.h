/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

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

#include <math_utility.h>

#include <Kin/taskMap.h>
#include <Kin/taskMaps.h>

using namespace std;

struct ShapePairCollisionConstraint:TaskMap
{
  ShapePairCollisionConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName, double _margin=.02)
  : margin_( _margin )
  {
    i_ = G.getShapeByName( iShapeName )->index;
    j_ = G.getShapeByName( jShapeName )->index;

    //std::cout << iShapeName << "--" << jShapeName << std::endl;
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t);

  void phiNoCollision( arr& y, arr& J, const mlr::KinematicWorld& G, mlr::Proxy * p );

  void phiCollision( arr& y, arr& J, const mlr::KinematicWorld& G );

  uint dim_phi(const mlr::KinematicWorld& G)
  {
    return 1;// + constantVectors_.N;
  }

  mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("_PairCollisionConstraint"); }

private:
  uint i_;
  uint j_;
  double margin_;
};
