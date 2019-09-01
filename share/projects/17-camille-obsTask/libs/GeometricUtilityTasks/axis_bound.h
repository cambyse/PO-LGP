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

#include <Kin/feature.h>
#include <Kin/taskMaps.h>

#include <Kin/proxy.h>

using namespace std;

struct AxisBound:Feature{

  enum Axis
  {
    X = 0,
    Y,
    Z
  };

  enum BoundType
  {
    MIN = 0,
    MAX,
    EQUAL
  };

  AxisBound( const std::string & object, double bound, const enum Axis & axis, const enum BoundType & boundType, const double k = 1.0 )
    : object_( object )
    , bound_( bound )
    , boundType_( boundType )
    , k_( k )
  {
    if( axis == X ) id_ = 0;
    else if( axis == Y ) id_ = 1;
    else if( axis == Z ) id_ = 2;
  }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G)
  {
//    for( auto p : G.proxies )
//    {
//      std::cout << p->a << " " << p->b << std::endl;
//    }

    rai::Frame *object = G.getFrameByName( object_.c_str() );
    arr posObject, posJObject;
    G.kinematicsPos(posObject, posJObject, object);    // get function to minimize and its jacobian in state G

    const double sign = ( ( boundType_ == MIN ) ? 1 : -1 );

    arr tmp_y = zeros( dim_ );
    tmp_y( 0 ) = - k_ * sign * ( posObject( id_ ) - bound_ );

    arr tmp_J = zeros( dim_, posJObject.dim(1) );
    tmp_J.setMatrixBlock( - k_ * sign * posJObject.row( id_ ), 0 , 0 );    // jacobian

    // commit results
    y = tmp_y;
    if(&J) J = tmp_J;
  }

  virtual uint dim_phi(const rai::KinematicWorld& G)
  {
    return dim_;
  }

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("AxisBound");
  }

private:
  static const uint dim_ = 1;
  std::string object_;
  const double bound_;
  const BoundType boundType_;
  const double k_;
  std::size_t id_= 0;
};
