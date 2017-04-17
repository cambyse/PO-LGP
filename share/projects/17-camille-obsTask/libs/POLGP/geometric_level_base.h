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

#pragma once

#include <sys/types.h>
#include "komo_factory.h"

class POLGPNode;

struct GeometricLevelBase
{
  typedef std::shared_ptr< GeometricLevelBase > ptr;

  GeometricLevelBase( const GeometricLevelBase& that ) = delete;  // no copy

  GeometricLevelBase( POLGPNode * node, std::string const& name, const KOMOFactory & komoFactory );

  std::string name_;

  uint N_;
  mlr::Array< double > costs_;          // optimization result costs ( one per world )
  mlr::Array< double > constraints_;    // optimization result costs ( one per world )
  mlr::Array< bool >   solved_;         // whether the optimization succedded or not ( one per world )
  mlr::Array< bool >   feasibles_;      // whether the optimization succedded or not ( one per world )
  mlr::Array< ExtensibleKOMO::ptr > komos_; // opti state after optimization ( one per world )
  bool isTerminal_;                     // terminal node and solved
  bool isSolved_;                       // is solved ( each possible world is solved )

  POLGPNode * node_;

  //-- komo factory
  const KOMOFactory & komoFactory_;

  virtual void solve() = 0;
  virtual void backtrack() = 0;

  // parameters
  double maxConstraints_ = 0.5;
  double maxCost_        = 7.5;
};

