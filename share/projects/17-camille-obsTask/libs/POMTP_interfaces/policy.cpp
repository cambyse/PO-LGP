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
#include <Core/graph.h>

static int policyNumber = 0;

//----PolicyNode---------------------//

void PolicyNode::save( std::ostream& os )
{
  // id
  os << "id" << std::endl;
  os << id_  << std::endl;

  // belief state size
  os << "size" << std::endl;
  os << bs_.size() << std::endl;

  // states
  os << "states:" << std::endl;
  for( auto w = 0; w < bs_.size(); w++ )
  {
    os << "w:" << w << std::endl;
    if( bs_( w ) > 0 )
    {
      states()( w )->write( os );
    }
  }

  os << "belief_state" << std::endl;
  for( auto b : bs_ )
  {
    os << b << std::endl;
  }

  os << "next_action" << std::endl;
  os << nextAction_ << std::endl;

  os << "time" << std::endl;
  os << time_  << std::endl;

  os << "p" << std::endl;
  os << p_  << std::endl;

  os << "q" << std::endl;
  os << q_  << std::endl;

  os << "g" << std::endl;
  os << g_  << std::endl;

  os << "h" << std::endl;
  os << h_  << std::endl;


  /*
        mlr::Array< std::shared_ptr<Graph> > states_;
        arr bs_;
        // action
        std::string nextAction_; // action to take at this node
        //
        double time_;
        uint id_;

        double p_;  // probability of reaching this node
        double q_;  // probability of reaching this node given that fact that its parent is reached
        double g_;  // cost so far
        double h_;  // future costs*/
}

void PolicyNode::load( std::istream& is )
{

}

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

void Policy::save( std::ostream& os )
{
  saveFrom( root_, os );
}

void Policy::load( std::istream& is )
{

}

void Policy::saveFrom( const PolicyNode::ptr & node, std::ostream& os )
{
  node->save( os );

  for( auto n : node->children() )
  {
    saveFrom( n, os );
  }
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

Policy::ptr fuse( Policy::ptr base, Policy::ptr over )
{
  over->root();
}


