#include <komo_planner.h>

#include <kin_equality_task.h>

#include <policy_visualizer.h>

namespace mp
{
static double eps() { return std::numeric_limits< double >::epsilon(); }

//--------Motion Planner--------------//

KOMOPlanner::KOMOPlanner()
{
  using namespace std::placeholders;

  //OverPlaneConstraintManager overPlane;

  //auto groundActivateOverPlane = std::bind( &OverPlaneConstraintManager::groundActivateOverPlane, &overPlane, _1, _2, _3, _4, _5 );
  //auto groundDeactivateOverPlane = std::bind( &OverPlaneConstraintManager::groundDeactivateOverPlane, &overPlane, _1, _2, _3, _4, _5 );
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
      //kin->watch( true );
      //
      startKinematics_.append( kin );
    }
  }
}

void KOMOPlanner::solveAndInform( const MotionPlanningOrder & po, Policy::ptr & policy )
{
  CHECK( startKinematics_.d0 == policy->N(), "consitency problem, the belief state size of the policy differs from the belief state size of the kinematics" );
  CHECK( po.policyId() == policy->id(), "id of the policy and the planning orders are not consistent" );

  po.getParam( "type" );

  if( po.getParam( "type" ) != "jointPath" )
  {
    CHECK( false, "not implemented yet!" );
  }

  clearLastPolicyOptimization();

  // solve on pose level
  //for( auto i = 0; i < 100; ++i )
  optimizePoses( policy );

  bool optimizationFailed = false;
  // if a node has a constraint which is not satisfied, we set the value g of the node to ineasible i.e. infinite cost!
  for( auto nodeConstraintsPair : poseConstraints_ )
  {
    auto node = nodeConstraintsPair.first;

    double maxConstraint = 0;
    for( auto constraint : nodeConstraintsPair.second )
    {
      maxConstraint = std::max( constraint, maxConstraint );
    }

    if( maxConstraint > 0.5 )
    {
      std::cout << "Optimization failed on node " << node->id() << std::endl;

      node->setG( std::numeric_limits< double >::infinity() );
      optimizationFailed = true;
    }
  }

  if( optimizationFailed )
  {
    policy->setCost( std::numeric_limits< double >::infinity() );
  }
  else
  {
    // solve on path level
    optimizePath( policy );

    // solve on joint path level
    optimizeJointPath( policy );

    // update policy cost
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
  }
  policy->setStatus( Policy::INFORMED );
}

void KOMOPlanner::display( const Policy::ptr & policy, double sec )
{
  Policy::ptr tmp( policy );
  MotionPlanningOrder po( policy->id() );
  po.setParam( "type", "jointPath" );
  // resolve since this planner doesn't store paths
  solveAndInform( po, tmp );

  // retrieve trajectories
  mlr::Array< mlr::Array< mlr::Array< mlr::KinematicWorld > > > frames;

  for( auto leafWorldKinFramesPair : jointPathKinFrames_ )
  {
    frames.append( leafWorldKinFramesPair.second );
  }

  PolicyVisualizer viz( frames, "policy" );

  mlr::wait( sec, true );
}

void KOMOPlanner::registerTask( const mlr::String & type, const SymbolGrounder & grounder )
{
  komoFactory_.registerTask( type, grounder );
}

void KOMOPlanner::clearLastPolicyOptimization()
{
  // poses
  effKinematics_.clear();
  poseConstraints_.clear();

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
  std::cout << "optimizing pose for:" << node->id() << std::endl;

  effKinematics_[ node ] = mlr::Array< mlr::KinematicWorld >( node->N() );
  poseCosts_      [ node ] = arr( node->N() );
  poseConstraints_[ node ] = arr( node->N() );
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
//        _komo->setTiming( 1., 2, secPerPhase, 1/*, true*/ );
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

      komo->setTiming( 1., 2, secPerPhase, 1/*, true*/ );
//      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
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

      poseCosts_[ node ]( w )       = cost;
      poseConstraints_[ node ]( w ) = constraints;

      // what to do with the cost and constraints here??

      // update effective kinematic
      effKinematics_[ node ]( w ) = *komo->configurations.last();

      // update switch
      for( mlr::KinematicSwitch *sw: komo->switches )
      {
        //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
        if( sw->timeOfApplication>=2 ) sw->apply( effKinematics_[ node ]( w ) );
      }
      effKinematics_[ node ]( w ).topSort();
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

void KOMOPlanner::optimizePath( Policy::ptr & policy )
{
  bsToLeafs_             = mlr::Array< PolicyNode::ptr > ( policy->N() );

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
      // indicate this leaf as terminal for this node, this is used during the joint optimization..
      bsToLeafs_( w ) = leaf;

      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *startKinematics_( w ), true, false, true, false, false );
      komo->setTiming( start_offset_ + leaf->time() + end_offset_, microSteps_, secPerPhase, 2/*, true*/ );

//      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??

      komo->setFixEffectiveJoints(-1., -1., fixEffJointsWeight);
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
      komo->setTiming( start_offset_ + leaf->time() + end_offset_, microSteps_, secPerPhase, 2/*, true*/ );

//      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??

      komo->setFixEffectiveJoints(-1., -1., fixEffJointsWeight);
      komo->setFixSwitchedObjects();
      komo->setSquaredQAccelerations();
      //komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      //komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        // set task
        auto time = ( node->parent() ? node->parent()->time(): 0. );   // get parent time

        komo->groundTasks( start_offset_ +  time, *node->states()( w ) );          // ground parent action (included in the initial state)

        if( node->time() > 0 )
        {
          uint stepsPerPhase = komo->stepsPerPhase; // get number of steps per phases
          uint nodeSlice = stepsPerPhase * ( start_offset_ + node->time() ) - 1;
          arr q = zeros( pathKinFrames_[ leaf ]( w )( nodeSlice ).q.N );

          // set constraints enforcing the path equality among worlds
          int nSupport = 0;
          for( auto x = 0; x < leaf->N(); ++x )
          {
            if( node->parent()->bs()( x ) > 0 )
            {
              CHECK( bsToLeafs_( x ) != nullptr, "no leaf for this state!!?" );

              auto terminalLeafx = bsToLeafs_( x );

              CHECK( pathKinFrames_[ terminalLeafx ]( x ).N > 0, "one node along the solution path doesn't have a path solution already!" );

              auto pathLeafx     = pathKinFrames_[ terminalLeafx ]( x );

              q += node->parent()->bs()( x ) * pathLeafx( nodeSlice ).q;

              nSupport++;
            }
          }

          if( nSupport > 1 )  // enforce kin equality between at least two worlds, useless with just one world!
          {
            AgentKinEquality * task = new AgentKinEquality( node->id(), q );  // tmp camille, think to delete it, or komo does it?
            double slice_t = start_offset_ + node->time() - 1.0 / stepsPerPhase;
            komo->setTask( slice_t, slice_t, task, OT_eq, NoArr, kinEqualityWeight  );

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
