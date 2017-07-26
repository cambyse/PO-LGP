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

#include "policy_builder.hpp"

namespace tp
{

//---PolicyVisualizer--------//
//PolicyVisualizer::PolicyVisualizer( const Policy::ptr & policy, const std::string & name )
//{
//  views_.resize( policy->N() );
//  for( auto w = 0; w < policy->N(); ++w )
//  {
//    std::string windowName = name + std::string( "-world-" ) + std::to_string( w );
//    views_[ w ] = std::make_shared< OrsPathViewer >( windowName.c_str(),  0.1, -0 );
//    views_[ w ]->setConfigurations( policy->getTrajectory( w ) );
//  }


//  threadOpenModules( true );
//}

//---PolicyBuilder-----------//

PolicyBuilder::PolicyBuilder( PONode * root )
  : policy_( std::make_shared< Policy >() )
{
  policy_->init( root->N() );

  process( root );
}

void PolicyBuilder::process( PONode * node )
{
  PolicyNode::ptr policyNode = std::make_shared< PolicyNode >();

  if( node->isRoot() )
  {
    policy_->setRoot( policyNode );
  }
  else
  {
    // set parent
    auto parent = PO2Policy_[ node->parent() ];
    policyNode->setParent( parent );

    // add child to parent
    parent->addChild( policyNode );
  }
  // set node data
  policyNode->setState( node->folStates(), node->bs() );
  policyNode->setNextAction( node->bestActionStr() );
  policyNode->setId( node->id() );
  policyNode->setDifferentiatingFact( node->differentiatingFacts() );

  // set node data
  policyNode->setTime( node->time() );
  policyNode->setP( node->pHistory() );

  if( node->parent() )
  {
    policyNode->setQ( node->pHistory() / node->parent()->pHistory() );
  }

  policyNode->setG( node->prefixReward() );
  policyNode->setH( node->expecteFutureReward() );

  // save correspondance
  PO2Policy_[ node ] = policyNode;

  for( auto c : node->bestFamily() )
  {
    process( c );
  }
}

Policy::ptr PolicyBuilder::getPolicy() const
{
  return policy_;
}

}

