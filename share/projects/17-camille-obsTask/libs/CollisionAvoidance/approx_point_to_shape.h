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

struct ApproxPointToShape:TaskMap
{
  ApproxPointToShape(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName, double radius = 0.05 )
    : radius_( radius )
  {
    i_ = G.getShapeByName( iShapeName )->index;
    j_ = G.getShapeByName( jShapeName )->index;

    mlr::Shape *a = G.shapes( i_ );
    mlr::Shape *b = G.shapes( j_ );

    CHECK( a->mesh_radius < 0.01, "The first shape should be almost a point!" );
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t);

  void phiProxy( arr& y, arr& J, const mlr::KinematicWorld& G, mlr::Proxy * p );

  uint dim_phi(const mlr::KinematicWorld& G)
  {
    return 1;
  }

  mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("_ApproxPointToShape"); }

private:
  uint i_;
  uint j_;
  double radius_;
};