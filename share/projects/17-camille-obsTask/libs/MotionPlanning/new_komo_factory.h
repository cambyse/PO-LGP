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

#include <KOMO/komo.h>

namespace mp
{

//=====ExtensibleKOMO==============================================
class NewExtensibleKOMO;

typedef std::function<void( double time, const std::vector< std::string >& facts, NewExtensibleKOMO *, int verbose )> SymbolGrounder;

class NewExtensibleKOMO : public KOMO
{

public:
  typedef std::shared_ptr< NewExtensibleKOMO > ptr;
public:
  NewExtensibleKOMO();

  void registerTask( const std::string & type, const SymbolGrounder & grounder );
  void groundTasks( double phase, const std::vector< std::string >& facts, int verbose=0 );

  void setPrefixSetup() { prefixSetup_ = true; }
  bool isPrefixSetup() const { return prefixSetup_; }

  void saveTrajectory( const std::string & suffix = "" ) const;
  void plotVelocity( const std::string & suffix = "" ) const;
  arr getCostsPerPhase();

private:
  std::map< std::string, SymbolGrounder > tasks_;
  bool prefixSetup_ = false; //
};

//=====ExtensibleKOMO==============================================

class NewKOMOFactory
{

public:
  void registerTask( const std::string & type, const SymbolGrounder & grounder );
  std::shared_ptr< NewExtensibleKOMO > createKomo() const;
private:
  std::map< std::string, SymbolGrounder > tasks_;
};

}
