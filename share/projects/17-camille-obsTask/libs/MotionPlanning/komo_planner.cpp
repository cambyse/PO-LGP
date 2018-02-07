#include <komo_planner.h>

#include <chrono>

#include <kin_equality_task.h>

#include <policy_visualizer.h>

#include <Kin/kin.h>
#include <Kin/switch.h>

namespace mp
{
static double eps() { return std::numeric_limits< double >::epsilon(); }

//--------Check integrity-------------//

bool checkPolicyIntegrity( const Policy::ptr & policy )
{
  bool isOk = true;

  std::list< PolicyNode::ptr > fifo;
  fifo.push_back( policy->root() );

  while( ! fifo.empty()  )
  {
    auto node = fifo.back();
    fifo.pop_back();

    if( ! node->children().empty() )
    {
      const auto r = node->children().first()->lastReward();

      for( auto c : node->children() )
      {
        isOk = isOk && r == c->lastReward();

        fifo.push_back( c );
      }
    }
  }

  return isOk;
}

static double updateValue( const PolicyNode::ptr & node )
{
  double value = 0;

  for( auto c : node->children() )
  {
    value += node->p() * c->q() * ( c->lastReward() + updateValue( c ) );
  }

  node->setValue( value );

  return value;
}

static void updateValues( Policy::ptr policy )
{
  policy->setValue( updateValue( policy->root() ) );
}

//--------Motion Planner--------------//

void KOMOPlanner::setKin( const std::string & kinDescription )
{
  Graph G( kinDescription.c_str() );

  if( G[ beliefStateTag_ ] == nullptr )
  {
    auto kin = std::make_shared< mlr::KinematicWorld >();
    kin->init( kinDescription.c_str() );
    computeMeshNormals( kin->frames );
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
        std::cout << *nn << std::endl;
        nn->newClone( kinG );
      }

      auto kin = std::make_shared< mlr::KinematicWorld >();
      kin->init( kinG );
      computeMeshNormals( kin->frames );
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

  /// EARLY STOPPING, detect if pose level not possible
  bool optimizationFailed = false;
  // if a node has a constraint which is not satisfied, we set the node to infeasible i.e. infinite cost!
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
      std::cout << "Optimization failed on node " << node->id() << " max constraint:" << maxConstraint << std::endl;

      node->setLastReward( std::numeric_limits< double >::lowest() );
      node->setValue( std::numeric_limits< double >::lowest() );
      optimizationFailed = true;
    }
  }  

  if( ! optimizationFailed )
  {
    // solve on path level
    optimizePath( policy );

    // solve on joint path level
    optimizeJointPath( policy );

    /// INFORM POLICY NODES
    std::list< PolicyNode::ptr > fifo;
    fifo.push_back( policy->root() );

    while( ! fifo.empty()  )
    {
      auto node = fifo.back();
      fifo.pop_back();

      auto phase = node->time();

      double cost = 0;

      // get the right world
      for( auto w = 0; w < node->N(); ++w )
      {
        if( node->bs()( w ) > 0 )
        {
          auto leaf = bsToLeafs_( w );
          CHECK( jointPathCostsPerPhase_.find( leaf ) != jointPathCostsPerPhase_.end(), "corruption in datastructure" );

          auto trajCosts = jointPathCostsPerPhase_[ leaf ]( w );
          auto wcost = trajCosts( phase_start_offset_ + phase );

          cost += node->bs()( w ) * wcost;
          //std::cout << "cost of phase:" << cost << " phase:" << phase << std::endl;
        }
      }

      // push children on list
      for( auto c : node->children() )
      {
        c->setLastReward( - cost );

        fifo.push_back( c );
      }
    }

    /// UPDATE VALUES
    updateValues( policy );

  }
  policy->setStatus( Policy::INFORMED );

  CHECK( checkPolicyIntegrity( policy ), "Policy is corrupted" );
}

void KOMOPlanner::display( const Policy::ptr & policy, double sec )
{
  Policy::ptr tmp( policy );
  MotionPlanningOrder po( policy->id() );
  po.setParam( "type", "jointPath" );
  // resolve since this planner doesn't store paths
  //
  auto start = std::chrono::high_resolution_clock::now();
  //

  solveAndInform( po, tmp );

  //
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  std::cout << "planning time:" << ms << std::endl;
  //

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

  effKinematics_  [ node ] = mlr::Array< mlr::KinematicWorld >( node->N() );
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

      komo->setTiming( 1., 2, 5., 1/*, true*/ );
//      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQVelocities();
      komo->setFixSwitchedObjects( -1., -1., 1e3 );

      komo->groundTasks( 0., *node->states()( w ) );

      komo->reset(); //huge

      try{
        komo->run();
      } catch( const char* msg ){
        cout << "KOMO FAILED: " << msg <<endl;
      }

//      if( node->id() == 136 )
//      {
//      komo->displayTrajectory();

//      mlr::wait();
//      }
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
      //effKinematics_[ node ]( w ).topSort();
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
  pathXSolution_[ leaf ] = mlr::Array< arr                               >( leaf->N() );

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
      komo->setTiming( phase_start_offset_ + leaf->time() + phase_end_offset_, microSteps_, secPerPhase_, 2 );

      //komo->setHoming( -1., -1., 1e-1 ); //gradient bug??

      komo->setFixEffectiveJoints(-1., -1., fixEffJointsWeight_ );
      komo->setFixSwitchedObjects();
      //komo->setSquaredQVelocities();
      komo->setSquaredQAccelerations();
      //komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      //komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        auto time = ( node->parent() ? node->parent()->time(): 0. );     // get parent time
        komo->groundTasks( phase_start_offset_ + time, *node->states()( w ) ); // ground parent action (included in the initial state)
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

//      if( leaf->id() == 136 )
//      {
//        komo->displayTrajectory();

//        mlr::wait();
//      }

      pathXSolution_[ leaf ]( w ) = komo->x;

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
  jointPathCostsPerPhase_[ leaf ] = mlr::Array< arr >( leaf->N() );

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
      komo->setTiming( phase_start_offset_ + leaf->time() + phase_end_offset_, microSteps_, secPerPhase_, 2 );

      komo->setFixEffectiveJoints(-1., -1., fixEffJointsWeight_ );
      komo->setFixSwitchedObjects();
      //komo->setSquaredQVelocities();
      komo->setSquaredQAccelerations();

      for( auto node:treepath )
      {
        // set task
        auto time = ( node->parent() ? node->parent()->time(): 0. );   // get parent time

        komo->groundTasks( phase_start_offset_ +  time, *node->states()( w ) );          // ground parent action (included in the initial state)

        if( node->time() > 0 )
        {
          for( auto s = 1; s < komo->stepsPerPhase; ++s )
          {
            uint stepsPerPhase = komo->stepsPerPhase; // get number of steps per phases
            uint nodeSlice = stepsPerPhase * ( phase_start_offset_ + node->time() ) - s;
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
              // retrieve agent joints
              auto G = *startKinematics_( w );

              uintA selectedBodies;

              for( const auto & f: G.frames )
              {
                if( f->ats["agent"] && f->ats.get<bool>("agent") )
                {
                  selectedBodies.setAppend(f->ID);
                }
              }

              // build mask
              arr qmask = zeros( G.q.d0 );

              for( auto b : selectedBodies )
              {
                mlr::Joint *j = G.frames.elem(b)->joint;

                CHECK( j, "incoherence, the joint should not be null since it has been retrieved before" );

                for( auto i = j->qIndex; i < j->qIndex + j->dim; ++i )
                {
                  qmask( i ) = 1;
                }
              }

              AgentKinEquality * task = new AgentKinEquality( node->id(), q, qmask );  // tmp camille, think to delete it, or komo does it?
              double slice_t = phase_start_offset_ + node->time() - double( s ) / stepsPerPhase;
              komo->setTask( slice_t, slice_t, task, OT_eq, NoArr, kinEqualityWeight_ );

              //
              //std::cout << slice_t << "->" << slice_t << ": kin equality " << std::endl;
              //
            }
          }
        }
      }

      komo->set_x( pathXSolution_[ leaf ]( w ) );
      komo->reset();

      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
//      if( w == 1 )
//      {
//        //komo->displayTrajectory( 0.02, true );
//        //komo->plotVelocity( std::to_string( w ) );
//      }

//      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
//      komo->checkGradients();
      auto costs = komo->getCostsPerPhase();
      Graph result = komo->getReport();
      //komo->getReport(true);

      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      // store costs
      jointPathCosts_      [ leaf ]( w )   = cost;
      jointPathConstraints_[ leaf ]( w )   = constraints;
      jointPathCostsPerPhase_[ leaf ]( w ) = costs;

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
