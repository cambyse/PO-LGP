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

#include <Math_utility/math_utility.h>

#include <Kin/taskMap.h>
#include <Kin/taskMaps.h>

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

using namespace std;

struct ShapePairFCL:TaskMap
{
  ShapePairFCL(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName )
  {
    CHECK( false, "do not use, fcl gives results that are not very usefull, to instable when colliding" );

    i_ = G.getShapeByName( iShapeName )->index;
    j_ = G.getShapeByName( jShapeName )->index;
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t);

  void phiProxy( arr& y, arr& J, const mlr::KinematicWorld& G, mlr::Proxy * p );

  void phiFCL( arr& y, arr& J, const mlr::KinematicWorld& G );

  uint dim_phi(const mlr::KinematicWorld& G)
  {
    return 1;// + constantVectors_.N;
  }

  mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("_ShapePairFCL"); }

private:
  fcl::CollisionObject * createObjectModel( mlr::Shape * s );

private:
  uint i_;
  uint j_;

  std::map< mlr::Shape *, std::vector<fcl::Vec3f> > vertices_;
  std::map< mlr::Shape *, std::vector<fcl::Triangle> > triangles_;
};
