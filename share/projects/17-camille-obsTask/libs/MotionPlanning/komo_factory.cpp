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

#include "komo_factory.h"

#include <Core/graph.h>

namespace mp
{

//==============KOMOFactory==============================================

void KOMOFactory::registerTask( const mlr::String & type, const SymbolGrounder & grounder )
{
  tasks_[ type ] = grounder;
}

std::shared_ptr< ExtensibleKOMO > KOMOFactory::createKomo() const
{
  auto komo = std::make_shared< ExtensibleKOMO >();
  for ( auto task : tasks_ )
  {
    komo->registerTask( task.first, task.second );
  }

  return komo;
}


//==============ExtensibleKOMO==============================================

ExtensibleKOMO::ExtensibleKOMO()
  : KOMO()
{

}

void ExtensibleKOMO::registerTask( const mlr::String & type, const SymbolGrounder & grounder )
{
  tasks_[ type ] = grounder;
}

void ExtensibleKOMO::groundTasks( double phase, const Graph& facts, int verbose )
{
  std::cout << "facts:" << facts << std::endl;

  for( Node *n:facts )
  {
    if( ! n->parents.N ) continue; // skip not relevant node

    if( n->keys.N && tasks_.count( n->keys.last() ) != 0 )
    {
      mlr::String type = n->keys.last();
      tasks_[ type ]( phase, facts, n, this, verbose ); // ground the symbol
    }
    else if( n->keys.N && n->keys.last().startsWith("komo") )
    {
      HALT("UNKNOWN komo TAG: '" <<n->keys.last() <<"'");
    }
  }
}

void ExtensibleKOMO::plotVelocity( const std::string & suffix )
{
  std::string filename = ( "z.velocities" + suffix ).c_str();
  std::string filenamePlt = ( "z.velocities" + suffix + ".plt" ).c_str();

  ofstream fil( filename.c_str() );
  StringA jointNames = world.getJointNames();
  //first line: legend
  for(auto s:jointNames) fil <<s <<' ';
  fil <<endl;

  // positions
  x.reshape(T, world.q.N);

  // speeds
  arr velocities = zeros(T-1, world.q.N);

  for( auto t = 0; t < T - 1; ++t )
  {
    auto x_t   = x.row( t );
    auto x_t_1 = x.row( t + 1 );
    auto v = x_t_1 - x_t;

    velocities.setMatrixBlock( v, t, 0 );
  }

  velocities.write(fil, NULL, NULL, "  ");
  fil.close();

  ofstream fil2( filenamePlt.c_str() );
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'velocities'" <<endl;
  fil2 <<"set term qt 2" <<endl;
  fil2 <<"plot '" << filename << "' \\" << endl;
  for(uint i=1;i<=jointNames.N;i++) fil2 <<(i>1?"  ,''":"     ") <<" u 0:"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
  //    if(dualSolution.N) for(uint i=0;i<tasks.N;i++) fil <<"  ,'' u 0:"<<1+tasks.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  // command
  std::string command = "load '" + filenamePlt + "'";
  gnuplot( command.c_str() );
}


}
