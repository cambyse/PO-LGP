#include <komo_planner.h>

#include <observation_tasks.h>
#include <object_pair_collision_avoidance.h>

#include <kin_equality_task.h>

#include <policy_visualizer.h>


namespace mp
{
static double eps() { return std::numeric_limits< double >::epsilon(); }

//------grounders------------//
void groundGrasp( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  if( *symbols(1) == "container_0" )
  {
    //disconnect object from table
    komo->setKinematicSwitch( t_end, true, "delete", "tableC", "container_0_bottom" );
    //connect graspRef with object
    komo->setKinematicSwitch( t_end, true, "ballZero", *symbols(0), "container_0_left" /**symbols(1)*/ );
  }
  else if( *symbols(1) == "container_1" )
  {
    //disconnect object from table
    komo->setKinematicSwitch( t_end, true, "delete", "tableC", "container_1_bottom" );
    //connect graspRef with object
    komo->setKinematicSwitch( t_end, true, "ballZero", *symbols(0), "container_1_left" /**symbols(1)*/ );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": grasping " << *symbols(1) << " with " << *symbols(0) << std::endl;
  }
}

void groundGraspObject( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_approach=phase + 0.25 * duration;
  const double t_switch =phase  + 0.5 * duration;
  const double t_end =   phase  + duration;
  //

  //komo->setTask( t_approach,    t_approach, new TaskMap_Default(posTMT, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.5}), OT_sumOfSqr, NoArr, 1e2);
  komo->setTask( t_approach, t_switch, new TaskMap_Default(posTMT,   komo->world, *symbols(0) ), OT_sumOfSqr, {0.,0.,-.2}, 1e1, 1);

  //komo->setTask( t_switch, t_end, new TaskMap_Default(posTMT, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.1}), OT_sumOfSqr, NoArr, 1e2);
  komo->setTask( t_switch, t_end, new TaskMap_Default(posTMT, komo->world, *symbols(0) ), OT_sumOfSqr, {0.,0.,.2}, 1e1, 1);


  //disconnect object from table
  komo->setKinematicSwitch( t_switch, true, "delete", NULL, *symbols(1) );
  //connect graspRef with object
  komo->setKinematicSwitch( t_switch, true, "ballZero", *symbols(0), *symbols(1) );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": grasping " << *symbols(1) << " with " << *symbols(0) << std::endl;
  }
}

void groundPlace( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;

  if( *symbols(1) == "container_0" )
  {
    komo->setPlace( t_end, *symbols(0), "container_0_front", *symbols(2), verbose );
  }
  else if( *symbols(1) == "container_1" )
  {
    komo->setPlace( t_end, *symbols(0), "container_1_front", *symbols(2), verbose );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " <<*symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;
  }
}

//void groundHome( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
//{
//  double duration=n->get<double>();

//  const double t = phase+duration;

//  //komo->setHoming( t, t + 1.0, 1e-2 ); //gradient bug??
//}

void groundGetSight( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  mlr::String arg = *symbols(0);

  komo->setTask( t_start, t_end, new ActiveGetSight      ( "manhead",
                                                                        arg,
                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                                        ARR( -0.0, 0.1, 0.4 ) ),  // pivot position  in container frame
                OT_sumOfSqr, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": getting sight of " << *symbols(0) << std::endl;
  }
}

void groundTakeView( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  auto *map = new TaskMap_Transition( komo->world );
  map->posCoeff = 0.;
  map->velCoeff = 1.;
  map->accCoeff = 0.;
  komo->setTask( t_start, t_end, map, OT_sumOfSqr, NoArr, 1e2, 1 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": taking view " << std::endl;
  }
}

//class OverPlaneConstraintManager
//{
//public:

void groundActivateOverPlane( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;//phase + 1.0;       // hack: the grasp task lasts 1 step, so we begin one step after
  const double t_end =   phase + duration; //komo->maxPhase;
  //

  if( *symbols(0) == "container_0" )
  {
    komo->setTask( t_start, t_end, new AxisAlignment( "container_0", ARR( 0, 0, 1.0 ) ), OT_eq, NoArr, 1e2 );
    komo->setTask( t_start, t_end, new AxisAlignment( "container_0", ARR( 1.0, 0, 0 ) ), OT_eq, NoArr, 1e2 );

    auto task = komo->setTask( t_start, t_end, new OverPlaneConstraint( komo->world, "container_0", *symbols(1), 0.05 ), OT_ineq, NoArr, 1e2 );

    //activeTasks_.push_back( ActiveTask{ komo, symbols, task } );
  }
  else if( *symbols(0) == "container_1" )
  {
    komo->setTask( t_start, t_end, new AxisAlignment( "container_1", ARR( 0, 0, 1.0 ) ), OT_eq, NoArr, 1e2 );
    komo->setTask( t_start, t_end, new AxisAlignment( "container_1", ARR( 1.0, 0, 0 ) ), OT_eq, NoArr, 1e2 );

    auto task = komo->setTask( t_start, t_end, new OverPlaneConstraint( komo->world, "container_1", *symbols(1), 0.05 ), OT_ineq, NoArr, 1e2 );

    //activeTasks_.push_back( ActiveTask{ komo, symbols, task } );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": over plane of " << *symbols(0) << " activated" << std::endl;
  }
}

/*void groundDeactivateOverPlane( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  for( auto req : activeTasks_ )
  {
    if( req.komo == komo && req.symbols == symbols )
    {
      //req.task->
    }
  }
}
private:

struct ActiveTask
{
  KOMO * komo;
  StringL symbols;
  Task * task;
};

std::list< ActiveTask > activeTasks_;

};*/


void groundObjectPairCollisionAvoidance( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  //std::cout << facts << std::endl;

  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =  komo->maxPhase;
  //

  for( auto s1 : komo->world.getBodyByName( *symbols(0) )->shapes )
  {
    for( auto s2 : komo->world.getBodyByName( *symbols(1) )->shapes )
    {
      //komo->setTask( t_start, t_end, new ShapePairCollisionConstraint( komo->world, s1->name, s2->name, 0.1 ), OT_ineq, NoArr, 1e2 );
    }
  }
}

//--------Motion Planner--------------//

KOMOPlanner::KOMOPlanner()
{
  using namespace std::placeholders;

  //OverPlaneConstraintManager overPlane;

  //auto groundActivateOverPlane = std::bind( &OverPlaneConstraintManager::groundActivateOverPlane, &overPlane, _1, _2, _3, _4, _5 );
  //auto groundDeactivateOverPlane = std::bind( &OverPlaneConstraintManager::groundDeactivateOverPlane, &overPlane, _1, _2, _3, _4, _5 );

  // register symbols
  komoFactory_.registerTask( "komoGrasp"       , groundGrasp );
  komoFactory_.registerTask( "komoGraspObject" , groundGraspObject );
  komoFactory_.registerTask( "komoPlace"       , groundPlace );
  komoFactory_.registerTask( "komoGetSight"    , groundGetSight );
  komoFactory_.registerTask( "komoTakeView"    , groundTakeView );
  komoFactory_.registerTask( "komoActivateOverPlane"   , groundActivateOverPlane );
  //komoFactory_.registerTask( "komoDeactivateOverPlane" , groundDeactivateOverPlane );
  komoFactory_.registerTask( "komoCollisionAvoidance", groundObjectPairCollisionAvoidance );
}

void KOMOPlanner::setKin( const std::string & kinDescription )
{
  Graph G( kinDescription.c_str() );

  if( G[ beliefStateTag_ ] == nullptr )
  {
    auto kin = std::make_shared< mlr::KinematicWorld >();
    kin->init( kinDescription.c_str() );
    computeMeshNormals( kin->shapes );
    kin->calc_fwdPropagateFrames();
    //kin->watch(/*true*/);

    startKinematics_.append( kin );
  }
  else
  {
    auto bsGraph = &G.get<Graph>( beliefStateTag_ );
    const uint nWorlds = bsGraph->d0;

    // build the different worlds
    for( uint w = 0; w < nWorlds; w++ )
    {
      Graph kinG( kinDescription.c_str() );

      // copy unobservable facts
      auto n = bsGraph->elem(w);

      for( auto nn : n->graph() )
      {
        nn->newClone( kinG );
      }

      auto kin = std::make_shared< mlr::KinematicWorld >();
      kin->init( kinG );
      computeMeshNormals( kin->shapes );
      kin->calc_fwdPropagateFrames();
      //
      //kin->watch(/*true*/);
      //
      startKinematics_.append( kin );
    }
  }
}

void KOMOPlanner::solveAndInform( Policy::ptr & policy )
{
  clearLastPolicyOptimization();

  // solve on pose level
  //for( auto i = 0; i < 100; ++i )
  optimizePoses( policy );

  // solve on path level
  optimizePath( policy );

  // solve on joint path level
  optimizeJointPath( policy );

  // update policy status
  double cost = 0;
  double constraints = 0;
  for( auto l : policy->leafs() )
  {
    for( auto w = 0; w < l->N(); ++w )
    {
      if( l->bs()( w ) > eps() )
      {
        cost        += jointPathCosts_      [ l ]( w ) * l->bs()( w );
        constraints += jointPathConstraints_[ l ]( w ) * l->bs()( w );
      }
    }
  }
  policy->setCost( cost );
  policy->setStatus( Policy::INFORMED );
}

void KOMOPlanner::clearLastPolicyOptimization()
{
  // poses
  effKinematics_.clear();

  // path
  for( auto pair : pathKinFrames_ )
  {
    pair.second.clear();
  }
  pathKinFrames_.clear(); // maps each leaf to its path

  // joint path
  jointPathCosts_.clear();
  jointPathConstraints_.clear();

  // path
  for( auto pair : jointPathKinFrames_ )
  {
    pair.second.clear();
  }
  jointPathKinFrames_.clear(); // maps each leaf to its path
}

void KOMOPlanner::optimizePoses( Policy::ptr & policy )
{
  optimizePosesFrom( policy->root() );
}

void KOMOPlanner::optimizePosesFrom( const PolicyNode::ptr & node )
{
  effKinematics_[ node ] = mlr::Array< mlr::KinematicWorld >( node->N() );
  //
  for( auto w = 0; w < node->N(); ++w )
  {
    if( node->bs()( w ) > eps() )
    {
      mlr::KinematicWorld kin = node->isRoot() ? *( startKinematics_( w ) ) : ( effKinematics_.find( node->parent() )->second( w ) );

//      ExtensibleKOMO::ptr _komo;

//      for( auto i = 0; i < 100; ++i )
//      {
//        _komo = komoFactory_.createKomo();
//        _komo->setModel( kin, true, false, true, false, false );
//        _komo->setTiming( 1., 2, 5., 1/*, true*/ );
//        _komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
//        _komo->setSquaredQVelocities();
//        _komo->setFixSwitchedObjects(-1., -1., 1e3);

//        _komo->groundTasks( 0., *node->states()( w ) );

//        _komo->reset(); //huge

//      }
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( kin, true, false, true, false, false );

      komo->setTiming( 1., 2, 5., 1/*, true*/ );
      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQVelocities();
      komo->setFixSwitchedObjects(-1., -1., 1e3);

      komo->groundTasks( 0., *node->states()( w ) );

      komo->reset(); //huge

      try{
        komo->run();
      } catch( const char* msg ){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      //komo->displayTrajectory();

      // save results
//    DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )

      Graph result = komo->getReport();

      double cost = result.get<double>( { "total","sqrCosts" } );
      double constraints = result.get<double>( { "total","constraints" } );

      // what to do with the cost and constraints here??

      // update effective kinematic
      //for( auto i = 0; i < 100; ++i )
      effKinematics_[ node ]( w ) = *komo->configurations.last();

      // update switch
      for( mlr::KinematicSwitch *sw: komo->switches )
      {
        //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
        if( sw->timeOfApplication>=2 ) sw->apply( effKinematics_[ node ]( w ) );
      }
      effKinematics_[ node ]( w ).topSort();
      //DEBUG( node->effKinematics()( w ).checkConsistency(); )
      effKinematics_[ node ]( w ).getJointState();

      // free
      freeKomo( komo );
    }
  }

  // solve for next nodes
  for( auto c : node->children() )
  {
    optimizePosesFrom( c );
  }
}

void KOMOPlanner::display( const Policy::ptr & policy, double sec )
{
  Policy::ptr tmp( policy );
  // resolve since this planner doesn't store paths
  solveAndInform( tmp );

  // retrieve trajectories
  mlr::Array< mlr::Array< mlr::Array< mlr::KinematicWorld > > > frames;

  for( auto leafWorldKinFramesPair : jointPathKinFrames_ )
  {
    frames.append( leafWorldKinFramesPair.second );
  }

  PolicyVisualizer viz( frames, "policy" );

  mlr::wait( sec, true );
}

void KOMOPlanner::optimizePath( Policy::ptr & policy )
{
  for( auto l : policy->leafs() )
  {
    optimizePathTo( l );
  }
}

void KOMOPlanner::optimizePathTo( const PolicyNode::ptr & leaf )
{
  pathKinFrames_[ leaf ] = mlr::Array< mlr::Array< mlr::KinematicWorld > >( leaf->N() );

  //-- collect 'path nodes'
  PolicyNode::L treepath = getPathTo( leaf );

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < leaf->N(); ++w )
  {
    if( leaf->bs()( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *startKinematics_( w ), true, false, true, false, false );
      komo->setTiming( start_offset_ + leaf->time() + end_offset_, microSteps_, 5., 2/*, true*/ );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??

      komo->setFixEffectiveJoints();
      komo->setFixSwitchedObjects();
      komo->setSquaredQAccelerations();
      //komo->setSquaredFixJointVelocities();// -1., -1., 1e3 );
      //komo->setSquaredFixSwitchedObjects();// -1., -1., 1e3 );

      for( auto node:treepath )
      {
        auto time = ( node->parent() ? node->parent()->time(): 0. );     // get parent time
        komo->groundTasks( start_offset_ + time, *node->states()( w ) ); // ground parent action (included in the initial state)
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

//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      //komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost        = result.get<double>( {"total","sqrCosts"} );
      double constraints = result.get<double>( {"total","constraints"} );

      for( auto s = 0; s < komo->configurations.N; ++s )
      {
        mlr::KinematicWorld kin( *komo->configurations( s ) );
        pathKinFrames_[ leaf ]( w ).append( kin );
      }

      // free
      freeKomo( komo );
    }
  }
}

void KOMOPlanner::optimizeJointPath( Policy::ptr & policy )
{
  for( auto l : policy->leafs() )
  {
    optimizeJointPathTo( l );
  }
}

void KOMOPlanner::optimizeJointPathTo( const PolicyNode::ptr & leaf )
{
  jointPathKinFrames_  [ leaf ] = mlr::Array< mlr::Array< mlr::KinematicWorld > >( leaf->N() );
  jointPathCosts_      [ leaf ] = arr( leaf->N() );
  jointPathConstraints_[ leaf ] = arr( leaf->N() );

  //-- collect 'path nodes'
  PolicyNode::L treepath = getPathTo( leaf );

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < leaf->N(); ++w )
  {
    if( leaf->bs()( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *startKinematics_( w ), true, false, true, false, false );
      komo->setTiming( start_offset_ + leaf->time() + end_offset_, microSteps_, 5., 2/*, true*/ );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??

      komo->setFixEffectiveJoints();
      komo->setFixSwitchedObjects();
      komo->setSquaredQAccelerations();
      //komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      //komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        // set task
        auto time = ( node->parent() ? node->parent()->time(): 0. );   // get parent time

        komo->groundTasks( start_offset_ +  time, *node->states()( w ), 1 );          // ground parent action (included in the initial state)

        if( node->time() > 0 )
        {
          uint stepsPerPhase = komo->stepsPerPhase; // get number of steps per phases
          uint nodeSlice = stepsPerPhase * ( start_offset_ + node->time() ) - 1;
          arr q = zeros( pathKinFrames_[ leaf ]( w )( nodeSlice ).q.N );

          // set constraints enforcing the path equality among worlds
          int nSupport = 0;
          for( auto x = 0; x < leaf->N(); ++x )
          {
            if( leaf->bs()( x ) > 0 )
            {
              CHECK( pathKinFrames_[ leaf ]( x ).N > 0, "one node along the solution path doesn't have a path solution already!" );

              q += node->bs()( x ) * pathKinFrames_[ leaf ]( x )( nodeSlice ).q;

              nSupport++;
            }
          }

          if( nSupport > 1 )  // enforce kin equality between at least two worlds, useless with just one world!
          {
            AgentKinEquality * task = new AgentKinEquality( node->id(), q );  // tmp camille, think to delete it, or komo does it?
            double slice_t = start_offset_ + node->time() - 1.0 / stepsPerPhase;
            komo->setTask( slice_t, slice_t, task, OT_eq, NoArr, 1e2  );

            //
            //std::cout << slice_t << "->" << slice_t << ": kin equality " << std::endl;
            //
          }
        }
      }

      komo->reset();
      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();

//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
//      komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      jointPathCosts_      [ leaf ]( w ) = cost;
      jointPathConstraints_[ leaf ]( w ) = constraints;

      // store result
      for( auto s = 0; s < komo->configurations.N; ++s )
      {
        mlr::KinematicWorld kin( *komo->configurations( s ) );
        jointPathKinFrames_[ leaf ]( w ).append( kin );
      }

      // free
      freeKomo( komo );
    }
  }
}

void freeKomo( ExtensibleKOMO::ptr komo )
{
  listDelete( komo->configurations );
  listDelete( komo->tasks );
  listDelete( komo->switches );
}

}
