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

struct Vars // Branch
{
  intA order0;
  intA order1;
  intA order2;
  uint microSteps;
  const uint k_order = 2;

  Vars(const intA & order0, const intA& order1, const intA& order2, uint microSteps)
    : order0(order0)
    , order1(order1)
    , order2(order2)
    , microSteps(microSteps)
  {

  }

  intA getVars(double from, double to, uint order) const
  {
    CHECK(from>=0.0, "invalid start time");
    CHECK(to<order0.d0*microSteps, "invalid end time");

    uint indexFrom = from * microSteps;
    uint indexTo = to > 0 ? to * microSteps : (*this)[order].d0;

    if(indexFrom > 1) indexFrom--;
    indexTo--;

    CHECK(indexFrom>=0, "invalid start index time");
    CHECK(indexTo<order0.d0, "invalid end index time");

    return (*this)[order].sub(indexFrom, indexTo, 0, -1);
  }

  int getStep(double time) const
  {
    return order0(time * microSteps - 1, 0);
  }

  const intA& operator[](std::size_t i) const
  {
    CHECK(i <= 2, "wrong order request!");
    switch(i)
    {
      case 0:
        return order0;
      case 1:
        return order1;
      case 2:
        return order2;
      default:
        break;
    }
  }
};

//=====ExtensibleKOMO==============================================
class ExtensibleKOMO;

typedef std::function<void( KOMO_ext*, int verbose )> InitGrounder;
typedef std::function<void( double time, const std::vector< std::string >& facts, KOMO_ext*, int verbose )> SymbolGrounder;
typedef std::function<void( double time, const Vars& vars, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )> TreeSymbolGrounder;

class ExtensibleKOMO : public KOMO_ext
{

public:
  typedef std::shared_ptr< ExtensibleKOMO > ptr;
public:
  ExtensibleKOMO();

  void registerInit( const InitGrounder & grounder );
  void groundInit( int verbose = 0 );

  void registerTask( const std::string & type, const SymbolGrounder & grounder );
  void registerTask( const std::string & type, const TreeSymbolGrounder & grounder );
  void groundTasks( double phase, const std::vector< std::string >& facts, int verbose=0 );
  void groundTasks( double start, const Vars& vars, const arr & scales, const std::vector< std::string >& facts, int verbose=0 );

  void applyRandomization( const std::vector< double > & randomVec );

  void saveTrajectory( const std::string & suffix = "" ) const;
  void plotVelocity( const std::string & suffix = "" ) const;
  arr getCostsPerPhase();

private:
  InitGrounder initGrounder_;
  std::map< std::string, SymbolGrounder > tasks_;
  std::map< std::string, TreeSymbolGrounder > treeTasks_;
};

//=====ExtensibleKOMO==============================================

class KOMOFactory
{

public:
  void registerInit( const InitGrounder & grounder );
  void registerTask( const std::string & type, const SymbolGrounder & grounder );
  void registerTask( const std::string & type, const TreeSymbolGrounder & grounder );
  std::shared_ptr< ExtensibleKOMO > createKomo() const;
private:
  InitGrounder initGrounder_;
  std::map< std::string, SymbolGrounder > tasks_;
  std::map< std::string, TreeSymbolGrounder > treeTasks_;
};

}
