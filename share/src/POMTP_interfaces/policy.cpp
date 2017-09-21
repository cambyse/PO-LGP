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

#include "policy.h"

static int policyNumber = 0;

//----Policy-------------------------//
Policy::Policy()
  : status_( SKELETON )
  , id_( policyNumber )
{
  policyNumber++;
}

void Policy::init( uint N )
{
  N_ = N;
}

//-----utility free functions-----------//
PolicyNode::L getPathTo( const PolicyNode::ptr & node )
{
  PolicyNode::L path;

  auto n = node;

  path.append( n );

  while( n->parent() )
  {
    path.append( n->parent() );
    n = n->parent();
  }

  path.reverse();

  return path;
}

bool policyCompare( Policy::ptr lhs, Policy::ptr rhs )
{
  return ! ( lhs->cost() == rhs->cost() ) && ( lhs->cost() < rhs->cost() );
}

bool skeletonEquals( Policy::ptr lhs, Policy::ptr rhs )
{
  auto ll = lhs->leafs();
  auto lr = rhs->leafs();

  bool equal = true;

  for( auto i = 0; i < ll.d0; ++i )
  {
    if( i < lr.d0 )
    {
      equal && ( ll(i)->id() == (lr(i))->id() );
    }
    else
    {
      equal = false;
    }
  }

  return equal;
}


