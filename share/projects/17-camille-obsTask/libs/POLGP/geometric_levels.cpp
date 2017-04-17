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

#include "geometric_levels.h"
#include "polgp_node.h"
#include "kin_equality_task.h"

static double eps() { return std::numeric_limits< double >::epsilon(); }
static double m_inf() { return -std::numeric_limits< double >::max(); }

// pose

PoseLevelType::PoseLevelType( POLGPNode * node, const KOMOFactory & komoFactory )
  : GeometricLevelBase( node, "pose", komoFactory )
{

}

void PoseLevelType::solve()
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = node_->getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( node_->bs()( w ) > eps() )
    {
      mlr::KinematicWorld kin = node_->isRoot() ? *node_->startKinematics()( w ) : node_->parent()->effKinematics()( w );

      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( kin );
      komo->setTiming( 1., 2, 5., 1, false );
      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQVelocities();
      komo->setSquaredFixSwitchedObjects(-1., -1., 1e3);

      if( node_->id() == 110 )
      {
        std::cout << *node_->folStates()( w ) << std::endl;
      }

      komo->groundTasks( 0., *node_->folStates()( w ) );

//      DEBUG( FILE("z.fol") << fol; )
//      DEBUG( komo->MP->reportFeatures( true, FILE( "z.problem" ) ); )
      komo->reset();
      try{
        komo->run();
      } catch( const char* msg ){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
      COUNT_evals += komo->opt->newton.evals;
      COUNT_kin += mlr::KinematicWorld::setJointStateCount;
      COUNT_poseOpt++;
      //poseCount++;

      // save results
//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )

      Graph result = komo->getReport();
//      DEBUG( FILE( "z.problem.cost" ) << result; )
      double cost = result.get<double>( { "total","sqrCosts" } );
      double constraints = result.get<double>( { "total","constraints" } );

      if( ! node_->isRoot() )
      {
        cost += node_->parent()->poseGeometricLevel()->costs_( w );
      }

      // if this pose leads to the smaller cost so far
      if( ! solved_( w ) || cost < costs_( w ) )
      {
        bool solved =  constraints< maxConstraints_ && cost < maxCost_;

//        if( ! solved )
//        {
//          std::cout << "!!!can't be solved for:" << node_->id() << " cost:" << cost << std::endl;
//        }
//        else
//        {
//          std::cout << "ok for:" << node_->id() << " cost:" << cost << std::endl;
//        }

        costs_( w ) = cost;
        constraints_( w ) = constraints;
        solved_( w )    = solved;
        feasibles_( w ) = solved;
        komos_( w ) = komo;

        // update effective kinematic
        node_->effKinematics()( w ) = *komos_( w )->MP->configurations.last();

        // update switch
        for( mlr::KinematicSwitch *sw: komos_( w )->MP->switches )
        {
          //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
          if( sw->timeOfApplication>=2 ) sw->apply( node_->effKinematics()( w ) );
        }
        node_->effKinematics()( w ).topSort();
        //DEBUG( node->effKinematics()( w ).checkConsistency(); )
        node_->effKinematics()( w ).getJointState();
      }

      // inform symbolic level
//      if( ! poseFeasibles_( w ) )
//        labelInfeasible();

    }
  }

  backtrack();
}

void PoseLevelType::backtrack()
{
  if( node_->isSymbolicallyTerminal() )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );

        if( ! feasibles_( w ) )
        {
          node_->labelInfeasible(); // label this sequence of actions as infeasible
        }
      }
    }

    isSolved_ = solved;

    if( isSolved_ )
    {
      std::cout << "Terminal node: " << node_->id() << " set solved" << std::endl;
    }

    if( isSolved_ )
    {
      isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );

        if( ! feasibles_( w ) )
        {
          node_->labelInfeasible(); // label this sequence of actions as infeasible
        }
      }
    }

    for( auto s : node_->bestFamily() )
    {
      solved = solved && s->poseGeometricLevel()->isSolved_;
    }

    isSolved_ = solved;

    if( isSolved_ )
    {
      std::cout << node_->id() << " set solved" << std::endl;
    }
  }

  // continue backtracking
  if( node_->parent() )
  {
    node_->parent()->poseGeometricLevel()->backtrack();
  }
}

// seq

SeqLevelType::SeqLevelType( POLGPNode * node, const KOMOFactory & komoFactory )
  : GeometricLevelBase( node, "seq", komoFactory )
{

}

void SeqLevelType::solve()
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = node_->getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( node_->bs()( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *node_->startKinematics()( w ) );
      komo->setTiming( node_->time(), 2, 5., 1, false );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQVelocities();
      komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        auto time = ( node->parent() ? node->parent()->time(): 0. ); // get parent time
        komo->groundTasks( time, *node->folStates()( w ) );        // ground parent action (included in the initial state)
      }

//      DEBUG( FILE("z.fol") << folWorlds_( w ); )
//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      komo->reset();

      try{
        komo->run();
      } catch(const char* msg){
        cout <<"KOMO FAILED: " <<msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
      COUNT_evals += komo->opt->newton.evals;
      COUNT_kin += mlr::KinematicWorld::setJointStateCount;
      COUNT_seqOpt++;

//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
          //  komo.checkGradients();

      Graph result = komo->getReport();
//      DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! komos_( w ) || cost < costs_( w ) )
      {
        bool solved =  constraints < maxConstraints_;

        costs_( w )       = cost;
        constraints_( w ) = constraints;
        solved_( w )      = solved;
        feasibles_( w )   = solved;
        komos_( w )       = komo;
      }
    }
  }

  backtrack();
}

void SeqLevelType::backtrack()
{
  if( node_->poseGeometricLevel()->isTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );
      }
    }
    isSolved_ = solved;

    if( isSolved_ )
    {
      isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : node_->bestFamily() )
    {
      solved = solved && s->seqGeometricLevel()->isSolved_;
    }

    isSolved_ = solved;
  }

  // continue backtracking
  if( node_->parent() )
  {
    node_->parent()->seqGeometricLevel()->backtrack();
  }
}

// path

PathLevelType::PathLevelType( POLGPNode * node, const KOMOFactory & komoFactory, uint microSteps )
  : GeometricLevelBase( node, "path", komoFactory )
  , microSteps_( microSteps )
{

}

void PathLevelType::solve()
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = node_->getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( node_->bs()( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *node_->startKinematics()( w ) );
      komo->setTiming( node_->time(), microSteps_, 5., 2, false );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQAccelerations();
      komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        auto time = ( node->parent() ? node->parent()->time(): 0. );   // get parent time
        komo->groundTasks( time, *node->folStates()( w ) );          // ground parent action (included in the initial state)
      }

//      DEBUG( FILE("z.fol") <<fol; )
//          DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
          komo->reset();
      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
      COUNT_evals += komo->opt->newton.evals;
      COUNT_kin += mlr::KinematicWorld::setJointStateCount;
      COUNT_pathOpt++;

//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      //komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! costs_( w ) || cost < costs_( w ) )     //
      {
        bool solved =  constraints< maxConstraints_;

        costs_( w )       = cost;                     //
        constraints_( w ) = constraints;              //
        solved_( w )      = solved;
        feasibles_( w )   = solved;                   //
        komos_( w )       = komo;                     //

        // back the best komo for this world
        for( POLGPNode * node = node_; node; node = node->parent() )
        {
          node->pathGeometricLevel()->komos_( w ) = komo;
        }
      }

//      if( ! pathFeasibles_( w ) )
//        labelInfeasible();
    }
  }

  backtrack();
}

void PathLevelType::backtrack()
{
  if( node_->seqGeometricLevel()->isTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );
      }
    }
    isSolved_ = solved;

    if( isSolved_ )
    {
      isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : node_->bestFamily() )
    {
      solved = solved && s->pathGeometricLevel()->isSolved_;
    }

    isSolved_ = solved;
  }

  // continue backtracking
  if( node_->parent() )
  {
    node_->parent()->pathGeometricLevel()->backtrack();
  }
}

// joint path

JointPathLevelType::JointPathLevelType( POLGPNode * node, const KOMOFactory & komoFactory, uint microSteps )
  : GeometricLevelBase( node, "joint path", komoFactory )
  , microSteps_( microSteps )
{

}

void JointPathLevelType::solve()
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = node_->getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( node_->bs()( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *node_->startKinematics()( w ) );
      komo->setTiming( node_->time(), microSteps_, 5., 2, false );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQAccelerations();
      komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        // set task
        auto time = ( node->parent() ? node->parent()->time(): 0. );   // get parent time
        komo->groundTasks( time, *node->folStates()( w ) );          // ground parent action (included in the initial state)

        if( node->time() > 0 )
        {
          uint pathMicroSteps = ( node_->pathGeometricLevel()->komos_( w )->MP->configurations.N - 1 ) / node_->time();
          uint nodeSlice = pathMicroSteps * node->time() - 1;
          arr q = zeros( node_->pathGeometricLevel()->komos_( w )->MP->configurations( nodeSlice )->q.N );

          // set constraints enforcing the path equality among worlds
          for( auto x = 0; x < N_; ++x )
          {
            if( node->bs()( x ) > 0 )
            {
              auto komo = node->pathGeometricLevel()->komos_( x );

              CHECK( node->pathGeometricLevel()->komos_( x )->MP->configurations.N > 0, "one node along the solution path doesn't have a path solution already!" );

              q += node->bs()( x ) * node->pathGeometricLevel()->komos_( x )->MP->configurations( nodeSlice )->q;
            }
          }

          AgentKinEquality * task = new AgentKinEquality( node->id(), q );  // tmp camille, think to delete it, or komo does it?

          //std::cout << "t:" << node->time_ << " q.N " << q.N  << std::endl;

          komo->setTask( node->time() - 1.0 / pathMicroSteps, node->time() - 1.0 / pathMicroSteps, task, OT_sumOfSqr, NoArr, 1e2  );

        }
      }

//      DEBUG( FILE("z.fol") <<fol; )
//      DEBUG( komo->MP->reportFeatures( true, FILE("z.problem") ); )

      komo->reset();
      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
      COUNT_evals += komo->opt->newton.evals;
      COUNT_kin += mlr::KinematicWorld::setJointStateCount;
      //COUNT_jointPathOpt++;

//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      //komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! costs_( w ) || cost < costs_( w ) )       //
      {
        bool solved =  constraints < maxConstraints_;

        costs_( w )       = cost;                     //
        constraints_( w ) = constraints;              //
        solved_( w )      = solved;         //
        feasibles_( w )   = solved;         //
        komos_( w )       = komo;
      }
      //      if( ! jointPathFeasibles_( w ) )
      //        labelInfeasible();
    }
  }

  backtrack();
}

void JointPathLevelType::backtrack()
{
  if( node_->pathGeometricLevel()->isTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( node_->bs()( w ) > eps() )
      {
        solved = solved && solved_( w );
      }
    }
    isSolved_ = solved;

    if( isSolved_ )
    {
      isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : node_->bestFamily() )
    {
      solved = solved && s->jointPathGeometricLevel()->isSolved_;
    }

    isSolved_ = solved;
  }

  // continue backtracking
  if( node_->parent() )
  {
    node_->parent()->jointPathGeometricLevel()->backtrack();
  }
}
