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

#include <Motion/taskMap.h>
#include <Motion/taskMaps.h>

using namespace std;

struct AxisAlignment:TaskMap
{
  AxisAlignment( const char* bobyName, const arr & axis )
    : bobyName_     ( bobyName )
    , axis_( axis )
  {

  }

  virtual void phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t );

  uint dim_phi( const mlr::KinematicWorld& G ) { return 1; }

  mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("AxisAlignment"); }

  private:
    const mlr::String bobyName_;
    const arr axis_;
};

struct OverPlaneConstraint:TaskMap
{
  OverPlaneConstraint( const mlr::KinematicWorld& G, const char* iBobyName, const char* jPlaneBodyName, double _margin=.02 )
    : iBobyName_     ( iBobyName )
    , jPlaneBodyName_( jPlaneBodyName )
    , margin_( _margin )
  {
    //collisionModel_.append( mlr::Vector( 0, 0, 0 ) );

    // tmp camille is only temporary, get voxels from the sahpe
    collisionModel_.append( mlr::Vector( -0.15, -0.15, 0 ) );
    collisionModel_.append( mlr::Vector( -0.15,  0.15, 0 ) );
    collisionModel_.append( mlr::Vector(  0.15, -0.15, 0 ) );
    collisionModel_.append( mlr::Vector(  0.15, 0.15, 0 ) );
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t)
  {
    auto body = G.getBodyByName( iBobyName_ );
    auto plane = G.getBodyByName( jPlaneBodyName_ );

    arr positionPLane;
    arr positionJPLane;
    G.kinematicsPos( positionPLane, positionJPLane, plane );

    arr tmp_y = zeros( collisionModel_.N );
    arr tmp_J = zeros( collisionModel_.N, positionJPLane.d1 );

    for( auto w = 0; w < collisionModel_.N; ++w )
    {
      arr _y;
      arr _Jy;
      G.kinematicsPos( _y, _Jy, body, collisionModel_( w ) );
//      std::cout << w << std::endl;
//      std::cout << _Jy << std::endl;

      double md = positionPLane( 2 ) -_y( 2 ) + margin_;

      tmp_y( w ) = md;

//      std::cout << "container_1:" << _y << std::endl;

//      std::cout << "table:" << positionTable << std::endl;
      //std::cout << "md:" << md << std::endl;

      for( auto i = 0; i < tmp_J.d1; ++i )
      {
          tmp_J( w, i ) = positionJPLane( 2, i ) - _Jy( 2, i );
      }
    }

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("OverTableConstraint"); }

  uint dim_phi(const mlr::KinematicWorld& G)
  {
    return collisionModel_.N;// + constantVectors_.N;
  }

private:
  mlr::String iBobyName_;
  mlr::String jPlaneBodyName_;
  double margin_;
  mlr::Array< mlr::Vector > collisionModel_;
  //mlr::Array< mlr::Vector > constantVectors_;

};

struct ShapePairCollisionConstraint:PairCollisionConstraint
{
  ShapePairCollisionConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName, double _margin=.02)
    : PairCollisionConstraint( G, iShapeName, jShapeName, _margin )
  {
    i_ = G.getShapeByName( iShapeName )->index;
    j_ = G.getShapeByName( jShapeName )->index;

    //std::cout << iShapeName << "--" << jShapeName << std::endl;
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t);

  mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("_PairCollisionConstraint"); }

private:
  uint i_;
  uint j_;
};
