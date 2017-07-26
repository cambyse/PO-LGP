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

//----Policy-------------------------//
Policy::Policy()
  : status_( SKELETON )
{

}

void Policy::init( uint N )
{
  N_ = N;
}

//----PolicyPrinter------------------//

void PolicyPrinter::print( const Policy::ptr & policy )
{
  ss_ << "digraph g{" << std::endl;

  printFromNode( policy->root() );

  ss_ << "}" << std::endl;
}

void PolicyPrinter::printFromNode( const PolicyNode::ptr & node )
{
  for( auto c : node->children() )
  {
    std::stringstream ss1;
    ss1 << node->nextAction();

    auto diffFacts = c->differentiatingFacts();

    for( auto fact : c->differentiatingFacts() )
    {
      ss1 << std::endl << fact;
    }

    if( node->children().N > 1 )
    {
      ss1 << std::endl << "p=" << c->p();
      ss1 << std::endl << "q=" << c->p() / node->p();
    }

    auto label = ss1.str();

    ss_ << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

    printFromNode( c );
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

/*RUN_ON_INIT_BEGIN(policy)
mlr::Array< PolicyNode::ptr >::memMove = true;
RUN_ON_INIT_END(policy)*/
