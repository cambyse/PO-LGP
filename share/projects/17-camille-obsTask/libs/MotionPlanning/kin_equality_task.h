/*  ------------------------------------------------------------------
    Copyright 2017 Camille Phiquepal
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

#include <Kin/taskMap.h>

namespace mp
{

struct AgentKinEquality:TaskMap{

  AgentKinEquality( uint id, const arr& q, const arr qmask )
    : id_( id )
    , q_  ( q )
    , qmask_( qmask )
    , dim_( q.N )
  {

  }

  virtual void phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1 )
  {
    y = zeros( dim_ );
    y.setVectorBlock( G.q - q_, 0 );

    auto tmp_J = eye( dim_, dim_ );

    // apply mask
    for( auto i = 0; i < qmask_.d0; ++i )
    {
      y( i ) = y( i ) * qmask_( i );
      tmp_J( i, i ) = tmp_J( i, i ) * qmask_( i );
    }

    if(&J) J = tmp_J;
  }

  virtual uint dim_phi( const mlr::KinematicWorld& G )
  {
    return dim_;
  }

  virtual mlr::String shortTag( const mlr::KinematicWorld& G )
  {
    return mlr::String( "AgentKinEquality" );
  }

private:
  // parameters
  const uint id_;
  const arr q_;
  const arr qmask_;
  const uint dim_;
};

}
