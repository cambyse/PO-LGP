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

bool policyCompare( Policy::ptr lhs, Policy::ptr rhs )
{
  return ! ( lhs->cost() == rhs->cost() ) && ( lhs->cost() < rhs->cost() );
}


