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

#include <memory>
#include <map>

#include <KOMO/komo-ext.h>

namespace mp
{

//=====ExtensibleKOMO==============================================
class ExtensibleKOMO;

typedef std::function<void( KOMO_ext*, int verbose )> InitGrounder;
typedef std::function<void( double time, const std::vector< std::string >& facts, KOMO_ext*, int verbose )> SymbolGrounder;

class ExtensibleKOMO : public KOMO
{

public:
  typedef std::shared_ptr< ExtensibleKOMO > ptr;
public:
  ExtensibleKOMO();

  void registerInit( const InitGrounder & grounder );
  //void groundInit( std::vector< double > randomVec, bool applyInitialRandoVector, int verbose = 0 );
  void groundInit( int verbose = 0 );

  void registerTask( const std::string & type, const SymbolGrounder & grounder );
  void groundTasks( double phase, const std::vector< std::string >& facts, int verbose=0 );

  void applyRandomization( const std::vector< double > & randomVec );

  void saveTrajectory( const std::string & suffix = "" ) const;
  void plotVelocity( const std::string & suffix = "" ) const;
  arr getCostsPerPhase();

private:
  InitGrounder initGrounder_;
  std::map< std::string, SymbolGrounder > tasks_;
};

//=====ExtensibleKOMO==============================================

class KOMOFactory
{

public:
  void registerInit( const InitGrounder & grounder );
  void registerTask( const std::string & type, const SymbolGrounder & grounder );
  std::shared_ptr< ExtensibleKOMO > createKomo() const;
private:
  InitGrounder initGrounder_;
  std::map< std::string, SymbolGrounder > tasks_;
};

}
