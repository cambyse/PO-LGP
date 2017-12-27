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
#include <unordered_map>
#include <functional>

static int policyNumber = 0;

//----PolicyNode---------------------//

void PolicyNode::exchangeChildren( const mlr::Array< PolicyNode::ptr > & children )
{
  if( children.empty() )
    return;

  // exchange the leading action
  setNextAction( children.first()->parent()->nextAction() );

  // exchange the children
  children_ = children;

  for( auto c : children )
  {
    c->setParent( shared_from_this() );
  }
}

PolicyNode::ptr PolicyNode::clone() const
{
  auto node = std::make_shared< PolicyNode >();

  mlr::Array< std::shared_ptr<Graph> > states;
  for( const auto & s_ : states_ )
  {
//    auto s = std::make_shared< Graph >();
//    s->copy( *s_ );
    states.append( s_ );
  }
  node->states_ = states;

  node->bs_ = bs_;
  node->nextAction_ = nextAction_;
  node->time_ = time_;
  node->id_ = id_;
  node->p_ = p_;
  node->q_ = q_;
  node->prefixReward_ = prefixReward_;
  node->value_ = value_;
  node->differentiatingFacts_ = differentiatingFacts_;

  return node;
}

void PolicyNode::cloneFrom( const PolicyNode::ptr & node ) const
{
  CHECK( id() == node->id(), "" );

  for( auto c_ : children_ )
  {
    auto c = c_->clone();
    node->addChild( c );
    c->setParent( node );

    c_->cloneFrom( c );
  }
}

//void PolicyNode::save( std::ostream& os )
//{
//  // id
//  os << "id" << std::endl;
//  os << id_  << std::endl;

//  // belief state size
//  os << "size" << std::endl;
//  os << bs_.size() << std::endl;

//  // states
//  os << "states:" << std::endl;
//  for( auto w = 0; w < bs_.size(); w++ )
//  {
//    os << "w:" << w << std::endl;
//    if( bs_( w ) > 0 )
//    {
//      states()( w )->write( os );
//    }
//  }

//  os << "belief_state" << std::endl;
//  for( auto b : bs_ )
//  {
//    os << b << std::endl;
//  }

//  os << "next_action" << std::endl;
//  os << nextAction_ << std::endl;

//  os << "time" << std::endl;
//  os << time_  << std::endl;

//  os << "p" << std::endl;
//  os << p_  << std::endl;

//  os << "q" << std::endl;
//  os << q_  << std::endl;

//  os << "g" << std::endl;
//  os << g_  << std::endl;

//  os << "h" << std::endl;
//  os << h_  << std::endl;


//  /*
//        mlr::Array< std::shared_ptr<Graph> > states_;
//        arr bs_;
//        // action
//        std::string nextAction_; // action to take at this node
//        //
//        double time_;
//        uint id_;

//        double p_;  // probability of reaching this node
//        double q_;  // probability of reaching this node given that fact that its parent is reached
//        double g_;  // cost so far
//        double h_;  // future costs*/
//}

//void PolicyNode::load( std::istream& is )
//{

//}

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

Policy::ptr Policy::clone() const
{
  auto policy = std::make_shared< Policy >();

  policy->N_ = N_;

  auto root = root_->clone();
  policy->setRoot( root );

  root_->cloneFrom( root );

  policy->value_ = value_;
  policy->status_ = status_;

  std::function< void( const PolicyNode::ptr & ) > updateLeafsFrom;
  updateLeafsFrom= [&policy, &updateLeafsFrom, this] ( const PolicyNode::ptr & node )
  {
    for( auto leaf : leafs_ )
    {
      if( node->id() == leaf->id() )
      {
        policy->addLeaf( node );
      }
    }

    for( auto c : node->children() )
    {
      updateLeafsFrom( c );
    }
  };

  updateLeafsFrom( root );

  return policy;
}

//void Policy::save( std::ostream& os )
//{
//  saveFrom( root_, os );
//}

//void Policy::load( std::istream& is )
//{

//}

//void Policy::saveFrom( const PolicyNode::ptr & node, std::ostream& os )
//{
//  node->save( os );

//  for( auto n : node->children() )
//  {
//    saveFrom( n, os );
//  }
//}

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
  return ! ( lhs->value() == rhs->value() ) && ( lhs->value() > rhs->value() );
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

static std::list< PolicyNode::ptr > serializeFrom( const PolicyNode::ptr & node )
{
  std::list< PolicyNode::ptr > nodes;

  nodes.push_back( node );

  for( auto n : node->children() )
  {
    auto newNodes = serializeFrom( n );
    nodes.insert( nodes.end(), newNodes.begin(), newNodes.end() );
  }

  return nodes;
}

std::list< PolicyNode::ptr > serialize( const Policy::ptr & policy )
{
  return serializeFrom( policy->root() );
}

std::list< std::list< PolicyNode::ptr > > segmentFrom( const PolicyNode::ptr & node )
{
//  std::list< PolicyNode::ptr > nodes;

//  if( node->children().size() == 1 )
//  { // on the same segment, we add the following nodes
//    auto newNodes = serializeFrom( n );
//    nodes.insert( nodes.end(), newNodes.begin(), newNodes.end() );
//  }
//  else
//  {

//  }
}

std::vector< std::list< PolicyNode::ptr > > segment( const Policy::ptr & policy )
{
  std::unordered_map< uint, std::list< PolicyNode::ptr > > segments;

  auto current = policy->root();

  std::list< std::pair< PolicyNode::ptr, uint > > nodeSegPairs;

  nodeSegPairs.push_back( std::make_pair( policy->root(), 0 ) );

  while( ! nodeSegPairs.empty() )
  {
    auto nodeSegPair = nodeSegPairs.back();
    nodeSegPairs.pop_back();

    auto current  = nodeSegPair.first;
    auto branchId = nodeSegPair.second;

    segments[ branchId ].push_back( current );

    if( current->children().size() == 1 )
    { // just one child, we put the current node on the same segment
      nodeSegPairs.push_back( { current->children().front(), branchId } );
    }
    else
    {
      for( auto c : current->children() )
      {
        branchId++;
        nodeSegPairs.push_back( { c, branchId } );
      }
    }
  }

  // build return vector
  std::vector< std::list< PolicyNode::ptr > > ret_segments( segments.size() );

  for( auto l : segments )
  {
    ret_segments[ l.first ] = l.second;
  }

  return ret_segments;
}

bool equivalent( const std::list< PolicyNode::ptr > & s1, const std::list< PolicyNode::ptr > & s2 )
{
  if( s1.size() != s2.size() )
    return false;

  bool equal = true;
  auto it1 = s1.begin();
  auto it2 = s2.begin();
  while( it1 != s1.end() )
  {
    equal = equal && ( (*it1)->id() == (*it2)->id() );
    ++it1;
    ++it2;
  }

  return equal;
}

static void setNewValueAndBackTrack( const PolicyNode::ptr & node, double newValue )
{
  auto p = node->parent();

  if( ! p )
  {
    node->setValue( newValue );
    return;
  }

  // original step Cost
  auto stepCost = node->value() - p->value();

  //
  node->setValue( newValue );

  double parentValue = -stepCost;
  for( auto c : p->children() )
  {
    parentValue += c->p() * c->value();
  }

  setNewValueAndBackTrack( p, parentValue );
}

Policy::ptr fuse( Policy::ptr base, Policy::ptr over )
{
  auto fused = base->clone();

  auto nodes = serialize( fused );

  // exchange children starting from the over node
  for( const auto & node : nodes )
  {
    if( node->id() == over->root()->id() )
    {
      auto valueNode = node->value();
      auto newValue  = over->root()->value();

      node->exchangeChildren( over->root()->children() );

      // update recursively the costs
      setNewValueAndBackTrack( node, newValue );
    }
  }

  // update leafs
  fused->resetLeafs();
  auto fused_nodes = serialize( fused );
  auto base_leafs = base->leafs();
  auto over_leafs = over->leafs();

  for( auto n : fused_nodes )
  {
    bool isLeaf = false;
    for( auto m : base_leafs )
    {
      if( m->id() == n->id() )
      {
        isLeaf = true;
      }
    }

    for( auto m : over_leafs )
    {
      if( m->id() == n->id() )
      {
        isLeaf = true;
      }
    }

    if( isLeaf )
    {
      fused->addLeaf( n );
    }
  }

  // update the costs
  fused->setValue( fused->root()->value() );

  return fused;
}


