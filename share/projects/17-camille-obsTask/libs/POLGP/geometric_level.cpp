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

#include "geometric_level.h"

GeometricLevelType::GeometricLevelType( uint N )
  : N_( N )
  , costs_( N )
  , constraints_( N )
  , solved_( N )
  , feasibles_( N )
  , komos_( N )
  , isTerminal_( false )
  , isSolved_( false )
{
  for( auto w = 0; w < N_; ++w )
  {
    costs_( w ) = 0;
    constraints_( w ) = 0;
    solved_( w ) = false;
    feasibles_( w ) = true;
  }
}

PoseLevelType::PoseLevelType( uint N )
  : GeometricLevelType( "pose level", N )
{

}

void PoseLevelType::solve()
{

}
