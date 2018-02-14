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

#include <policy_printer.h>

//----PolicyPrinter------------------//

void PolicyPrinter::print( const Policy::ptr & policy )
{
  ss_ << "digraph g{" << std::endl;

  ss_ << "label=" << "\"value=" << policy->value() << "\"" << std::endl;
  ss_ << "labelloc=top" << std::endl;

  printFromNode( policy->root() );

  ss_ << "}" << std::endl;
}

void PolicyPrinter::printFromNode( const PolicyNode::ptr & node )
{
  if( node->children().size() == 1 )
  {
    auto c = node->children().first();

    std::stringstream ss1;
    ss1 << node->nextAction();
    ss1 << std::endl << "r=" << c->lastReward();

    auto label = ss1.str();

    ss_ << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

    printFromNode( c );
  }
  else if( node->children().size() > 1 )
  {
    auto c = node->children().first();

    std::stringstream ss1;
    ss1 << node->nextAction();

    auto label1 = ss1.str();

    std::string oid = std::to_string( node->id() ) + "'";
    ss_ << "\"" << oid << "\"" << " [ " << "shape=box" << " ] " << ";" << std::endl;
    ss_ << node->id() << "->" << "\"" << oid << "\"" << " [ label=\"" << label1 << "\" ]" << ";" << std::endl;

    for( auto c : node->children() )
    {
      std::stringstream ss2;

      auto diffFacts = c->differentiatingFacts();

      for( auto fact : c->differentiatingFacts() )
      {
        ss2 << std::endl << fact;
      }

      ss2 << std::endl << "r=" << c->lastReward();
      ss2 << std::endl << "p=" << c->p();
      ss2 << std::endl << "q=" << c->p() / node->p();

      auto label2 = ss2.str();

      ss_ << "\"" << oid << "\"" << "->" << c->id() << " [ label=\"" << label2 << "\" ]" << ";" << std::endl;

      printFromNode( c );
    }
  }
//  for( auto c : node->children() )
//  {
//    std::stringstream ss1;
//    ss1 << node->nextAction();

//    if( node->children().N > 1 )
//    {
//      auto diffFacts = c->differentiatingFacts();

//      for( auto fact : c->differentiatingFacts() )
//      {
//        ss1 << std::endl << fact;
//      }

//      ss1 << std::endl << "p=" << c->p();
//      ss1 << std::endl << "q=" << c->p() / node->p();
//    }

//    ss1 << std::endl << "r=" << c->lastReward();

//    auto label = ss1.str();

//    ss_ << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

//    printFromNode( c );
//  }
}
