#include <komo_planner.h>

#include <chrono>

#include <kin_equality_task.h>

#include <trajectory_tree_visualizer.h>

#include <Kin/kin.h>
#include <Kin/switch.h>

namespace mp
{
static double eps() { return std::numeric_limits< double >::epsilon(); }
static arr extractAgentQMask( const mlr::KinematicWorld & G )  // retrieve agent joints
{
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

  return qmask;
}

static double updateValue( const Skeleton::GraphNodeType::ptr & node )
{
  double value = 0;

  for( auto c : node->children() )
  {
    value += c->data().p * ( c->data().markovianReturn + updateValue( c ) );
  }

  return value;
}

static void updateValues( Skeleton & policy )
{
  policy.setValue( updateValue( policy.root() ) );
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
        //std::cout << *nn << std::endl;
        nn->newClone( kinG );
      }

      auto kin = std::make_shared< mlr::KinematicWorld >();
      kin->init( kinG );
      computeMeshNormals( kin->frames );
      kin->calc_fwdPropagateFrames();
      //
      //if( w == 1 )
      //kin->watch( true );
      //
      startKinematics_.append( kin );
    }
  }
}

std::vector< double > KOMOPlanner::drawRandomVector( const std::vector< double > & override )
{
  if( startKinematics_.size() == 0 )
  {
    return std::vector< double >();
  }

  if( override.size() > 0 )
  {
    randomVec_ = override;
    return randomVec_;
  }

  auto world = startKinematics_(0);

  // get size of random Vector
  uint randomVecSize = 0;
  for( auto f : world->frames )
  {
    if( f->ats["random_bounds"]  )
    {
      auto randomBounds = f->ats.get<arr>("random_bounds");

      for( auto b : randomBounds )
      {
        if( b > 0 )
        {
          randomVecSize++;
        }
      }
    }
  }

  // draw it
  randomVec_ = std::vector< double >( randomVecSize );

  for( auto i = 0; i < randomVecSize; ++i )
  {
    auto v = rnd.uni(-1.0, 1.0);
    randomVec_[i] = v;
  }

  return randomVec_;
}

void KOMOPlanner::solveAndInform( const MotionPlanningParameters & po, Skeleton & policy )
{
  CHECK( startKinematics_.d0 == policy.N(), "consitency problem, the belief state size of the policy differs from the belief state size of the kinematics" );
  CHECK( po.policyId() == policy.id(), "id of the policy and the planning orders are not consistent" );

  //po.getParam( "type" );

  clearLastNonMarkovianResults();

  // solve on pose level
  optimizePoses( policy );

  /// EARLY STOPPING, detect if pose level not possible
  bool poseOptimizationFailed = false;
  // if a node has a constraint which is not satisfied, we set the node to infeasible i.e. infinite cost!

  {
    std::list< Skeleton::GraphNodeTypePtr > fifo;
    fifo.push_back( policy.root() );

    while( ! fifo.empty()  )
    {
      auto node = fifo.back();
      fifo.pop_back();

      double maxConstraint = 0;
      for( auto constraint : poseConstraints_[ node->data().decisionGraphNodeId ] )
      {
        maxConstraint = std::max( constraint, maxConstraint );
      }

      if( maxConstraint >= maxConstraint_ )
      {
        std::cout << "Pose Optimization failed on node " << node->id() << " max constraint:" << maxConstraint << std::endl;

        node->data().markovianReturn = std::numeric_limits< double >::lowest();
        //node->setValue( std::numeric_limits< double >::lowest() );
        //node->data().setStatus( PolicyNode::INFORMED );

        poseOptimizationFailed = true;
        policy.setValue( std::numeric_limits< double >::lowest() );
      }
      else
      {
        // push children on list
        for( auto c : node->children() )
        {
          fifo.push_back( c );
        }
      }
    }

    policy.setStatus( Skeleton::SKELETON );
  }

  if( poseOptimizationFailed )  // early stopping
    return;

  /// PATH OPTI
  if( po.getParam( "type" ) == "markovJointPath" )
  {
    optimizeMarkovianPath( policy );

    std::list< Skeleton::GraphNodeTypePtr > fifo;
    fifo.push_back( policy.root() );

    while( ! fifo.empty()  )
    {
      auto node = fifo.back();
      fifo.pop_back();

      double constraint = markovianPathConstraints_[ node->data().decisionGraphNodeId ];

      if( constraint >= maxConstraint_ )
      {
        std::cout << "Markovian Optimization failed on node " << node->id() << " constraint:" << constraint << std::endl;

        node->data().markovianReturn = std::numeric_limits< double >::lowest();
        //node->setValue( std::numeric_limits< double >::lowest() );
        //node->setStatus( PolicyNode::INFORMED );

        poseOptimizationFailed = true;
        policy.setValue( std::numeric_limits< double >::lowest() );
      }
      else
      {
        node->data().markovianReturn =  -( minMarkovianCost_ + markovianPathCosts_[ node->data().decisionGraphNodeId ] );
        //node->setStatus( PolicyNode::INFORMED );

        // push children on list
        for( auto c : node->children() )
        {
          fifo.push_back( c );
        }
      }
    }

    /// UPDATE VALUES
    updateValues( policy );

    policy.setStatus( Skeleton::INFORMED );
  }
  else if( po.getParam( "type" ) == "jointPath" )
  {
    // solve on path level
    optimizePath( policy );

    // solve on joint path level
    if( policy.N() > 1 )
    {
      optimizeJointPath( policy );
    }

    /// INFORM POLICY NODES
    std::list< Skeleton::GraphNodeTypePtr > fifo;
    fifo.push_back( policy.root() );

    while( ! fifo.empty()  )
    {
      auto node = fifo.back();
      fifo.pop_back();

      auto phase = node->depth() * 1.0;

      double cost = 0;

      // get the right world
      auto & pathCostsPerPhase = policy.N() > 1 ? jointPathCostsPerPhase_ : pathCostsPerPhase_;
      for( auto w = 0; w < node->data().beliefState.size(); ++w )
      {
        if( node->data().beliefState[ w ] > 0 )
        {
          auto leaf = bsToLeafs_( w );
          CHECK( pathCostsPerPhase.find( leaf ) != pathCostsPerPhase.end(), "corruption in datastructure" );

          auto trajCosts = pathCostsPerPhase[ leaf ]( w );
          auto wcost = trajCosts( phase_start_offset_ + phase );

          cost += node->data().beliefState[ w ] * wcost;
          //std::cout << "cost of phase:" << cost << " phase:" << phase << std::endl;
        }
      }

      // push children on list
      for( auto c : node->children() )
      {
        c->data().markovianReturn = - cost;

        fifo.push_back( c );
      }
    }

    /// UPDATE VALUES
    updateValues( policy );

    policy.setStatus( Skeleton::INFORMED );
  }
  else
  {
    CHECK( false, "not implemented yet!" );
  }

  //CHECK( checkPolicyIntegrity( policy ), "Policy is corrupted" );
}

void KOMOPlanner::display( const Skeleton & policy, double sec )
{
  Skeleton tmp( policy );
  MotionPlanningParameters po( policy.id() );
  po.setParam( "type", "jointPath" );
  // resolve since this planner doesn't store paths
  //
  auto start = std::chrono::high_resolution_clock::now();
  //

  solveAndInform( po, tmp );

  //
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  std::cout << "motion planning time (ms):" << ms << std::endl;
  //

  // retrieve trajectories
  mlr::Array< mlr::Array< mlr::Array< mlr::KinematicWorld > > > frames;

  const auto & kinFrames = policy.N() > 1 ? jointPathKinFrames_ : pathKinFrames_;
  for( auto leafWorldKinFramesPair : kinFrames )
  {
    frames.append( leafWorldKinFramesPair.second );
  }

  // display
  if( sec > 0 )
  {
    TrajectoryTreeVisualizer viz( frames, "policy" );

    mlr::wait( sec, true );
  }
}
std::pair< double, double > KOMOPlanner::evaluateLastSolution()
{
  // retrieve trajectories
  mlr::Array< mlr::Array< mlr::Array< mlr::KinematicWorld > > > frames;

  //CHECK( jointPathKinFrames_.size() == 0, "not supported yet if branching!" );

  const auto & kinFrames = jointPathKinFrames_.size() > 1 ? jointPathKinFrames_ : pathKinFrames_;
  for( auto leafWorldKinFramesPair : kinFrames )
  {
    frames.append( leafWorldKinFramesPair.second );
  }
  // evaluation
  for( auto k = 0; k < frames.N; ++k )
  {
    for( auto l = 0; l < frames.at(k).N; ++l )
    {
      auto eval = evaluate( frames.at(k).at(l), secPerPhase_ / microSteps_ );

      auto length = eval.first;
      auto acc_cost = eval.second;

      return std::make_pair( length, acc_cost );
    }
  }
}

void KOMOPlanner::registerInit( const InitGrounder & grounder )
{
  komoFactory_.registerInit( grounder );
}

void KOMOPlanner::registerTask( const std::string & type, const SymbolGrounder & grounder )
{
  komoFactory_.registerTask( type, grounder );
}

///MARKOVIAN

void KOMOPlanner::optimizePoses( Skeleton & policy )
{
  std::cout << "optimizing poses.." << std::endl;

  optimizePosesFrom( policy.root() );
}

void KOMOPlanner::optimizePosesFrom( const Skeleton::GraphNodeTypePtr & node )
{
  //std::cout << "optimizing pose for:" << node->id() << std::endl;

  bool feasible = true;

  const auto N = node->data().beliefState.size();
  //
  effKinematics_  [ node->data().decisionGraphNodeId ] = mlr::Array< mlr::KinematicWorld >( N );
  poseCosts_      [ node->data().decisionGraphNodeId ] = arr( N );
  poseConstraints_[ node->data().decisionGraphNodeId ] = arr( N );
  //
  for( auto w = 0; w < N; ++w )
  {
    if( node->data().beliefState[ w ] > eps() )
    {
      mlr::KinematicWorld kin = node->isRoot() ? *( startKinematics_( w ) ) : ( effKinematics_.find( node->parent()->data().decisionGraphNodeId )->second( w ) );

      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( kin, true, false, true, false, false );

      komo->setTiming( 1., 2, 5., 1/*, true*/ );
      //      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQVelocities();
      komo->setFixSwitchedObjects( -1., -1., 1e3 );

      komo->groundInit();
      komo->groundTasks( 0., node->data().leadingKomoArgs );

      if( node->isRoot() ) komo->applyRandomization( randomVec_ );
      komo->reset(); //huge

      try{
        komo->run();
      } catch( const char* msg ){
        cout << "KOMO FAILED: " << msg <<endl;
      }
      //komo->checkGradients();
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

      poseCosts_[ node->data().decisionGraphNodeId ]( w )       = cost;
      poseConstraints_[ node->data().decisionGraphNodeId ]( w ) = constraints;

      // what to do with the cost and constraints here??
      if( constraints >= maxConstraint_ )
      {
        feasible = false;
      }

      // update effective kinematic
      effKinematics_[ node->data().decisionGraphNodeId ]( w ) = *komo->configurations.last();

      // update switch
      for( mlr::KinematicSwitch *sw: komo->switches )
      {
        //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
        if( sw->timeOfApplication>=2 ) sw->apply( effKinematics_[ node->data().decisionGraphNodeId ]( w ) );
      }
      //effKinematics_[ node ]( w ).topSort();
      effKinematics_[ node->data().decisionGraphNodeId ]( w ).getJointState();

      // free
      freeKomo( komo );
    }
  }

  // solve for next nodes if this one was feasible
  if( feasible )
  {
    for( auto c : node->children() )
    {
      optimizePosesFrom( c );
    }
  }
}

// markovian path
void KOMOPlanner::optimizeMarkovianPath( Skeleton & policy )
{
  std::cout << "optimizing markovian paths.." << std::endl;

  optimizeMarkovianPathFrom( policy.root() );
}

void KOMOPlanner::optimizeMarkovianPathFrom( const Skeleton::GraphNodeTypePtr & node )
{
  //std::cout << "optimizing markovian path for:" << node->id() << std::endl;

  bool feasible = true;

  if( markovianPathCosts_.find( node->data().decisionGraphNodeId ) == markovianPathCosts_.end() )
  {
    markovianPathCosts_      [ node->data().decisionGraphNodeId ] = 0;
    markovianPathConstraints_[ node->data().decisionGraphNodeId ] = 0;

    const auto N = node->data().beliefState.size();

    for( auto w = 0; w < N; ++w )
    {
      if( node->data().beliefState[ w ] > eps() )
      {
        mlr::KinematicWorld kin = node->isRoot() ? *( startKinematics_( w ) ) : ( effKinematics_.find( node->parent()->data().decisionGraphNodeId )->second( w ) );
        // create komo
        auto komo = komoFactory_.createKomo();

        // set-up komo
        komo->setModel( kin, true, false, true, false, false );

        komo->setTiming( /*phase_start_offset_ + */1.0 + phase_end_offset_, microSteps_, secPerPhase_, 2 );

        komo->setFixEffectiveJoints(-1., -1., fixEffJointsWeight_ );
        komo->setFixSwitchedObjects();
        komo->setSquaredQAccelerations();

        komo->groundInit();
        komo->groundTasks( /*phase_start_offset_ +*/  0, node->data().leadingKomoArgs );

        if( node->isRoot() ) komo->applyRandomization( randomVec_ );
        komo->reset(); //huge

        try{
          komo->run();
        } catch( const char* msg ){
          cout << "KOMO FAILED: " << msg <<endl;
        }

//        if( node->id() == 2 )
//        {
////          komo->displayTrajectory();
////          komo->saveTrajectory( std::to_string( node->id() ) );
////          komo->plotVelocity( std::to_string( node->id() ) );

//          //mlr::wait();
//        }

        Graph result = komo->getReport();

        double cost = result.get<double>( { "total","sqrCosts" } );
        double constraints = result.get<double>( { "total","constraints" } );

        markovianPathCosts_      [ node->data().decisionGraphNodeId ] += node->data().beliefState[ w ] * cost;
        markovianPathConstraints_[ node->data().decisionGraphNodeId ] += node->data().beliefState[ w ] * constraints;

        // what to do with the cost and constraints here??
        if( constraints >= maxConstraint_ )
        {
          feasible = false;
        }

        // free
        freeKomo( komo );
      }
    }
  }
  // solve for next nodes if this one was feasible
  if( feasible )
  {
    for( auto c : node->children() )
    {
      optimizeMarkovianPathFrom( c );
    }
  }
}

///NON MARKOVIAN
void KOMOPlanner::clearLastNonMarkovianResults()
{
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
// path
void KOMOPlanner::optimizePath( Skeleton & policy )
{
  std::cout << "optimizing full path.." << std::endl;

  bsToLeafs_             = mlr::Array< PolicyNodePtr > ( policy.N() );

  for( auto l : policy.leafs() )
  {
    optimizePathTo( l.lock() );
  }
}

void KOMOPlanner::optimizePathTo( const PolicyNodePtr & leaf )
{
  const auto N = leaf->data().beliefState.size();

  pathKinFrames_[ leaf ] = mlr::Array< mlr::Array< mlr::KinematicWorld > >( N );
  pathXSolution_[ leaf ] = mlr::Array< arr                               >( N );
  pathCostsPerPhase_[ leaf ] = mlr::Array< arr >( N );

  //-- collect 'path nodes'
  auto treepath = getPathTo( leaf );

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N; ++w )
  {
    if( leaf->data().beliefState[ w ] > eps() )
    {
      // indicate this leaf as terminal for this node, this is used during the joint optimization..
      bsToLeafs_( w ) = leaf;

      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      auto leafTime = leaf->depth();
      komo->setModel( *startKinematics_( w ), true, false, true, false, false );
      komo->setTiming( phase_start_offset_ + leafTime + phase_end_offset_, microSteps_, secPerPhase_, 2 );

      komo->setFixEffectiveJoints(-1., -1., fixEffJointsWeight_ );
      komo->setFixSwitchedObjects();
      //komo->setSquaredQVelocities();
      komo->setSquaredQAccelerations();
      //komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      //komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      komo->groundInit();

      for( auto node:treepath )
      {
        auto time = ( node->parent() ? node->parent()->depth(): 0. );     // get parent time
        komo->groundTasks( phase_start_offset_ + time, node->data().leadingKomoArgs ); // ground parent action (included in the initial state)
      }

//      DEBUG( FILE("z.fol") <<fol; )
//          DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      komo->applyRandomization( randomVec_ );
      komo->reset();
      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
//      if( leaf->id() == 10 )
//      {
//        //      komo->plotTrajectory();
//        komo->displayTrajectory( 0.02, true );
//        komo->saveTrajectory( "-j-" + std::to_string( w ) );
//        komo->plotVelocity( "-j-"   + std::to_string( w ) );
//      }
      //komo->getReport(true);

      auto costs = komo->getCostsPerPhase();
      Graph result = komo->getReport();
      double cost        = result.get<double>( {"total","sqrCosts"} );
      double constraints = result.get<double>( {"total","constraints"} );

      pathCostsPerPhase_[ leaf ]( w ) = costs;

      for( auto s = 0; s < komo->configurations.N; ++s )
      {
        mlr::KinematicWorld kin( *komo->configurations( s ) );
        pathKinFrames_[ leaf ]( w ).append( kin );
      }

      pathXSolution_[ leaf ]( w ) = komo->x;

      // free
      freeKomo( komo );
    }
  }
}

void KOMOPlanner::optimizeJointPath( Skeleton & policy )
{
  std::cout << "optimizing full joint path.." << std::endl;

  for( auto l : policy.leafs() )
  {
    optimizeJointPathTo( l.lock() );
  }
}

void KOMOPlanner::optimizeJointPathTo( const PolicyNodePtr & leaf )
{
  const auto N = leaf->data().beliefState.size();

  jointPathKinFrames_  [ leaf ] = mlr::Array< mlr::Array< mlr::KinematicWorld > >( N );
  jointPathCosts_      [ leaf ] = arr( N );
  jointPathConstraints_[ leaf ] = arr( N );
  jointPathCostsPerPhase_[ leaf ] = mlr::Array< arr >( N );

  //-- collect 'path nodes'
  auto treepath = getPathTo( leaf );

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N; ++w )
  {
    if( leaf->data().beliefState[ w ] > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      auto leafTime = leaf->depth();
      komo->setModel( *startKinematics_( w ), true, false, true, false, false );
      komo->setTiming( phase_start_offset_ + leafTime + phase_end_offset_, microSteps_, secPerPhase_, 2 );

      komo->setFixEffectiveJoints(-1., -1., fixEffJointsWeight_ );
      komo->setFixSwitchedObjects();
      //komo->setSquaredQVelocities();
      komo->setSquaredQAccelerations();

      komo->groundInit();

      for( auto node:treepath )
      {
        // set task
        auto time = ( node->parent() ? node->parent()->depth(): 0. );     // get parent time
        komo->groundTasks( phase_start_offset_ + time, node->data().leadingKomoArgs );          // ground parent action (included in the initial state)

        if( node->depth() > 0 )
        {
          for( auto s = 1; s < komo->stepsPerPhase; ++s )
          {
            uint stepsPerPhase = komo->stepsPerPhase; // get number of steps per phases
            uint nodeSlice = stepsPerPhase * ( phase_start_offset_ + node->depth() ) - s;
            arr q = zeros( pathKinFrames_[ leaf ]( w )( nodeSlice ).q.N );

            // set constraints enforcing the path equality among worlds
            uint nSupport = 0;
            for( auto x = 0; x < N; ++x )
            {
              if( node->data().beliefState[ x ] > 0 )
              {
                CHECK( bsToLeafs_( x ) != nullptr, "no leaf for this state!!?" );

                auto terminalLeafx = bsToLeafs_( x );

                CHECK( pathKinFrames_[ terminalLeafx ]( x ).N > 0, "one node along the solution path doesn't have a path solution already!" );

                const auto & pathLeafx     = pathKinFrames_[ terminalLeafx ]( x );

                CHECK_EQ( q.N, pathLeafx( nodeSlice ).q.N, "wrong q dimensions!" );

                q += node->data().beliefState[ x ] * pathLeafx( nodeSlice ).q;

                nSupport++;
              }
            }

            if( nSupport > 1 )  // enforce kin equality between at least two worlds, useless with just one world!
            {
              arr qmask = extractAgentQMask( *startKinematics_( w ) );

              AgentKinEquality * task = new AgentKinEquality( node->id(), q, qmask );  // tmp camille, think to delete it, or komo does it?
              double slice_t = phase_start_offset_ + node->depth() - double( s ) / stepsPerPhase;
              komo->setTask( slice_t, slice_t, task, OT_eq, NoArr, kinEqualityWeight_ );

              //
              //std::cout << "depth:" << node->depth() << " slice:" << slice_t << " has kin equality, q size = " << qmask.size() << std::endl;
              //
            }
          }
        }
      }

      komo->applyRandomization( randomVec_ );
      komo->set_x( pathXSolution_[ leaf ]( w ) );
      komo->reset();

      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }

      // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
      //komo->displayTrajectory();
//     if( leaf->id() == 7 )
//     {
//  //      komo->plotTrajectory();
//        komo->displayTrajectory( 0.02, true );
//        komo->saveTrajectory( "-j-" + std::to_string( w ) );
//        komo->plotVelocity( "-j-"   + std::to_string( w ) );
//     }

      auto costs = komo->getCostsPerPhase();
      Graph result = komo->getReport();

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
