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

#include "new_komo_factory.h"

#include <Core/graph.h>

namespace mp
{

//==============KOMOFactory==============================================

void NewKOMOFactory::registerTask( const std::string & type, const SymbolGrounder & grounder )
{
  tasks_[ type ] = grounder;
}

std::shared_ptr< NewExtensibleKOMO > NewKOMOFactory::createKomo() const
{
  auto komo = std::make_shared< NewExtensibleKOMO >();
  for ( auto task : tasks_ )
  {
    komo->registerTask( task.first, task.second );
  }

  return komo;
}


//==============NewExtensibleKOMO==============================================

NewExtensibleKOMO::NewExtensibleKOMO()
  : KOMO()
{

}

void NewExtensibleKOMO::registerTask( const std::string & type, const SymbolGrounder & grounder )
{
  tasks_[ type ] = grounder;
}

void NewExtensibleKOMO::groundTasks( double phase, const std::vector< std::string >& facts, int verbose )
{
  //std::cout << "facts:" << facts << std::endl;

  auto type = facts.front();

  std::vector< std::string >args;
  if( facts.size() > 1 )
  {
    args = std::vector< std::string > { facts.begin() + 1, facts.end() };
  }

  if( tasks_.find( type ) != tasks_.end() )
  {
    tasks_[ type ]( phase, args, this, verbose ); // ground the symbol
  }
  else
  {
    HALT("UNKNOWN komo TAG: '" << type <<"'");
  }

//  for( Node *n:facts )
//  {
//    if( ! n->parents.N ) continue; // skip not relevant node

//    if( n->keys.N && tasks_.count( n->keys.last() ) != 0 )
//    {
//      mlr::String type = n->keys.last();
//      tasks_[ type ]( phase, facts, n, this, verbose ); // ground the symbol
//    }
//    else if( n->keys.N && n->keys.last().startsWith("komo") )
//    {
//      HALT("UNKNOWN komo TAG: '" <<n->keys.last() <<"'");
//    }
//  }
}

void NewExtensibleKOMO::saveTrajectory( const std::string & suffix ) const
{
  std::string filename = ( "z.coordinates" + suffix ).c_str();

  ofstream fil( filename.c_str() );
  StringA jointNames = world.getJointNames();
  //first line: legend
  for(auto s:jointNames) fil <<s <<' ';
  fil <<endl;

  // positions
  auto xx = x;
  xx.reshape(T, world.q.N);

  // coordinate
  arr coordinates = zeros(T, world.q.N);

  for( auto t = 0; t < T; ++t )
  {
    auto x_t   = xx.row( t );

    coordinates.setMatrixBlock( x_t, t, 0 );
  }

  coordinates.write(fil, NULL, NULL, "  ");
  fil.close();
}

void NewExtensibleKOMO::plotVelocity( const std::string & suffix ) const
{
  std::string filename = ( "z.velocities" + suffix ).c_str();
  std::string filenamePlt = ( "z.velocities" + suffix + ".plt" ).c_str();

  ofstream fil( filename.c_str() );
  StringA jointNames = world.getJointNames();
  //first line: legend
  for(auto s:jointNames) fil <<s <<' ';
  fil <<endl;

  // positions
  auto xx = x;
  xx.reshape(T, world.q.N);

  // speeds
  arr velocities = zeros(T-1, world.q.N);

  for( auto t = 0; t < T - 1; ++t )
  {
    auto x_t   = xx.row( t );
    auto x_t_1 = xx.row( t + 1 );
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

arr NewExtensibleKOMO::getCostsPerPhase()
{
  bool wasRun = featureValues.N!=0;

  arr phi;
  ObjectiveTypeA tt;
  if(wasRun){
      phi.referTo( featureValues.scalar() );
      tt.referTo( featureTypes.scalar() );
  }

  //-- collect all task costs and constraints
  StringA name; name.resize(tasks.N);
  arr err=zeros(maxPhase);
  uint M=0;
  for(uint t=0; t<T; t++){
    uint p = std::floor( t / stepsPerPhase );
    for(uint i=0; i<tasks.N; i++) {
      Task *task = tasks(i);
      if(task->prec.N>t && task->prec(t)){
        uint d=0;
        if(wasRun){
          d=task->map->dim_phi(configurations({t,t+k_order}), t);
          for(uint j=0;j<d;j++) CHECK(tt(M+j)==task->type,"");
          if(d){
            if(task->type==OT_sumOfSqr){
              for(uint j=0;j<d;j++) err(p) += mlr::sqr(phi(M+j)); //sumOfSqr(phi.sub(M,M+d-1));
            }
            if(task->type==OT_ineq){
              for(uint j=0;j<d;j++) err(p) += mlr::MAX(0., phi(M+j));
            }
            if(task->type==OT_eq){
              for(uint j=0;j<d;j++) err(p) += fabs(phi(M+j));
            }
            M += d;
          }
        }
      }
    }
  }
  CHECK_EQ(M , phi.N, "");

  return err;
}


}
