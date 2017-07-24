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

void Policy::init( uint N )
{
  N_ = N;
}

//----PolicyPrinter------------------//

void PolicyPrinter::print( const Policy::ptr & policy )
{
  ss_ << "digraph g{" << std::endl;

  //printPolicy( root_, ss );

  ss_ << "}" << std::endl;
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


