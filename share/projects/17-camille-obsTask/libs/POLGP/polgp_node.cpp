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


#include "polgp_node.h"

#include <unordered_map>

#include <list>

#include <boost/algorithm/string/replace.hpp>

#include <MCTS/solver_PlainMC.h>

#include <kin_equality_task.h>

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) x

uint COUNT_kin=0;
uint COUNT_evals=0;
uint COUNT_poseOpt=0;
uint COUNT_seqOpt=0;
uint COUNT_pathOpt=0;

//=====================free functions======================
static double eps() { return std::numeric_limits< double >::epsilon(); }
static double m_inf() { return -std::numeric_limits< double >::max(); }

struct stringSetHash {
size_t operator()( const std::set< std::string > & facts ) const
{
  std::string cont;
  for( auto s : facts )
    cont += s;

  return std::hash<std::string>()( cont );
}
};

static std::string toStdString( Node * node )
{
  std::stringstream ss;
  ss << * node;
  return ss.str();
}

static std::set< std::string > getObservableStateStr( Graph * state )
{
  // look for potential partial observability, we iterate over each fact
  std::set< std::string > facts;
  for( auto node : * state )
  {
    //std::cout << * node << std::endl;

    if( ( node->parents.first()->keys.first() == "NOT_OBSERVABLE" ) )
    {
      //std::cout << "is not observable!!" << node->parents.first()->keys.d0 << std::endl;
    }
    else
    {
      std::stringstream ss;
      ss << * node;
      facts.insert( ss.str() );
    }
  }

  return facts;
}

//===========================================================================

static int nodeNumber = 0;

/// root node init
POLGPNode::POLGPNode( mlr::Array< std::shared_ptr< FOL_World > > fols, const mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > & kins, const arr & bs, const KOMOFactory & komoFactory )
  : parent_( nullptr )
  , N_( fols.N )
  , folWorlds_( fols )
  , folStates_( N_ )
  //, folAddToStates_( N_ )
  , startKinematics_( kins )
  , effKinematics_( N_ )
  , pHistory_( 1.0 )
  , bs_( bs )
  , a_( -1 )
  , d_( 0 )
  , time_( 0.0 )
  // global search
  , isExpanded_( false )
  , isInfeasible_( false )
  //, isTerminal_( false )
  //, isSolved_( false )
  // logic search
  , isSymbolicallyTerminal_( false )
  , isSymbolicallySolved_( false )
  , rootMCs_( N_ )
  , mcStats_( new MCStatistics )
  , expectedReward_( m_inf() )
  , expectedBestA_( -1 )
  , komoFactory_( komoFactory )
  // poseOpt
  , poseProblem_( N_ )
  // seqOpt
  , seqProblem_( N_ )
  // pathOpt
  , pathProblem_( N_ )
  // jointPathOpt
  , jointProblem_( N_ )
  //
  , id_( 0 )
{
  for( auto w = 0; w < N_; ++w )
  {
    folWorlds_( w )->reset_state();
    folStates_( w ).reset( folWorlds_( w )->createStateCopy() );
    //folAddToStates_( w ) = nullptr;
    rootMCs_( w ).reset( new PlainMC( *folWorlds_( w ) ) );
    rootMCs_( w )->verbose = 0;
  }

  for( auto w = 0; w < N_; ++w )
  {
    effKinematics_( w ) = mlr::KinematicWorld( * startKinematics_( w ) );
  }

  std::size_t s = 0;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      s++;
    }
  }
}

/// child node creation
POLGPNode::POLGPNode(POLGPNode *parent, double pHistory, const arr & bs, uint a )
  : parent_( parent )
  , N_( parent_->N_ )
  , folWorlds_( parent->folWorlds_ )
  , folStates_( N_ )
  //, folAddToStates_( N_ )
  , startKinematics_( parent->startKinematics_ )
  , effKinematics_( parent->effKinematics_ )
  , decisions_( N_ )
  , pHistory_( pHistory )
  , bs_( bs )
  , a_( a )
  , d_( parent->d_ + 1 )
  // global search
  , isExpanded_( false )
  , isInfeasible_( false )
  //, isTerminal_( false )
  //, isSolved_( false )
  // logic search
  , isSymbolicallyTerminal_( false )
  , isSymbolicallySolved_( false )
  , rootMCs_( parent->rootMCs_ )
  , mcStats_( new MCStatistics )
  , expectedReward_( m_inf() )
  , expectedBestA_ (-1 )
  , komoFactory_( parent->komoFactory_ )
  // pose problem
  , poseProblem_( N_ )
  // seqOpt
  , seqProblem_( N_ )
  // pathOpt
  , pathProblem_( N_ )
  // jointPathOpt
  , jointProblem_( N_ )
{
  // update the states
  bool isTerminal = true;
  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      // logic
      auto fol = folWorlds_( w );
      //fol->reset_state();
      fol->setState( parent->folStates_( w ).get(), parent_->d_ );
      auto actions = fol->get_actions();

      fol->transition( actions[ a_ ] );

      folStates_( w ).reset( fol->createStateCopy() );

      bool isSubNodeTerminal = fol->successEnd;
      isTerminal = isTerminal && isSubNodeTerminal;

//      if( isTerminal )
//        std::cout << "found terminal " << std::endl;

      if( fol->deadEnd )
        isInfeasible_ = true;
      //std::cout << *folStates_( w ) << std::endl;

      //folAddToStates_( w ) = nullptr;

      decisions_( w ) = actions[ a_ ];
    }
  }
  isSymbolicallyTerminal_ = isTerminal;

  if( isTerminal )
    isSymbolicallySolved_ = true;

  // update time
  auto ls = getWitnessLogicAndState();
  time_ = parent_->time_ + ls.logic->lastStepDuration;

  // update support size
  std::size_t s = 0;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      s++;
    }
  }

  // change this
  nodeNumber++;
  id_ = nodeNumber;
}

void POLGPNode::expand()
{
  // debug
  //std::cout << std::endl;
  //std::cout << "POLGPNode::expand().." << std::endl;
  //auto ls = getWitnessLogicAndState();
  //std::cout << "observable state:" << getObservableStateStr( ls.state.get() ) << std::endl;
  //

  CHECK( ! isExpanded_, "" );
  if( isSymbolicallyTerminal_ )
    return;

  // get possible actions for the worlds having a non null probability
  auto nActions = getPossibleActionsNumber();

  //std::cout << "number of possible actions:" << nActions << std::endl;

  if( nActions == 0 )
    isSymbolicallyTerminal_ = true;

  for( auto a = 0; a < nActions; ++a )
  {
    //std::cout << "------------" << std::endl;
    //std::cout << "action:" << a << std::endl;
    std::unordered_map< std::set< std::string >, std::list< uint >, stringSetHash > outcomesToWorlds;

    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        auto logic = folWorlds_( w );
        auto state = folStates_( w );

        logic->setState( state.get() );

        auto actions = logic->get_actions();
        auto action = actions[ a ];

        logic->transition( action );

        auto result = logic->getState();
        auto observableStateStr = getObservableStateStr( result );

        outcomesToWorlds[ observableStateStr ].push_back( w );
      }
    }

    //std::cout << outcomesToWorlds.size() << " possible outcomes" << std::endl;

    // compute the observable facts intersection
    std::set< std::string > intersection = outcomesToWorlds.begin()->first;
    for( auto outcome = ++outcomesToWorlds.begin(); outcome != outcomesToWorlds.end(); ++outcome )
    {
      auto facts  = outcome->first;
      std::set< std::string > inter;
      std::set_intersection( intersection.begin(), intersection.end(),
                             facts.begin(), facts.end(),
                             std::inserter( inter, inter.begin() ) );
      intersection = inter;
    }

    // create as many children as outcomes
    mlr::Array< POLGPNode * > familiy;
    for( auto outcome : outcomesToWorlds )
    {
      auto facts  = outcome.first;
      auto worlds = outcome.second;

      // update belief state
      arr bs = zeros( N_ );
      double pWorld = 0;
      for( auto w : worlds )
      {
        pWorld += bs_( w );
        bs( w ) = bs_( w );
      }

      bs = bs / pWorld;

      CHECK( pWorld > 0, "wrong node expansion" );

      // create a node for each possible outcome

      auto n = new POLGPNode( this, pWorld * pHistory_, bs, a );
      familiy.append( n );

      // get the fact not in intersection
      std::set< std::string > differenciatingFacts;
      std::set_difference( facts.begin(), facts.end(), intersection.begin(), intersection.end(),
                           std::inserter(differenciatingFacts, differenciatingFacts.begin() ) );

      n->indicateDifferentiatingFacts( differenciatingFacts );
      //std::cout << "history:" << pHistory << " belief state:" << bs << " family size:" << familiy.d0 << std::endl;
    }

    // check integrity
    double pSum = 0;
    for( auto n : familiy ) pSum += n->pHistory();

    CHECK_ZERO( pSum / pHistory() - 1.0, 0.000001, "" );
    //

    families_.append( familiy );

    // indicate and relation
    for( auto n : familiy )
      n->setAndSiblings( familiy );
  }

  isExpanded_ = true;
}

void POLGPNode::setAndSiblings( const mlr::Array< POLGPNode * > & siblings )
{
  for( auto s : siblings )
  {
    if( s != this )
    andSiblings_.append( s );
  }
}

void POLGPNode::generateMCRollouts( uint num, int stepAbort )
{
  //std::cout << "POLGPNode::generateMCRollouts.." << std::endl;
  // do rollouts for each possible worlds
  auto treepath = getTreePath();

  arr R;  // rewards of all the rollouts

  for( auto w = 0; w < N_; ++w )
  {
    auto fol = folWorlds_( w );
    auto state = folStates_( w );
    auto rootMC = rootMCs_( w );
    // retrieve history
    if( bs_( w ) > eps() )
    {
      //std::cout << "world=" << w << std::endl;

      mlr::Array<MCTS_Environment::Handle> prefixDecisions( treepath.N-1 );

      for(uint i=1;i<treepath.N;i++)
        prefixDecisions(i-1) = treepath(i)->decision( w );

      for( uint k=0; k<num; ++k )
      {
        fol->reset_state();
        rootMC->initRollout( prefixDecisions );
        fol->setState( state.get() );
        double r = rootMC->finishRollout( stepAbort );
        R.append( bs_( w ) * r );
      }
    }
  }

  // save result
  double averageReward = 0;
  for( auto r: R )
  {
    averageReward += r;
    mcStats_->add( r );
  }

  averageReward /= num;

  // commit result
  expectedReward_ = averageReward;

  //std::cout << "average reward:" << expectedReward_ << std::endl;
}

void POLGPNode::backTrackBestExpectedPolicy()
{
  if( isSymbolicallyTerminal() )
  {
    expectedReward_ = 0;
    expectedBestA_  = -1;
  }
  else
  {
    struct familyStatusType { double reward; bool solved; };
    mlr::Array< familyStatusType > familyStatus( families_.d0 );

    // compute family costs
    for( auto i = 0; i < families_.d0; ++i )
    {
      double familyReward = 0;
      bool familySolved = true;
      for( auto c : families_( i ) )
      {
        familyReward += c->pHistory_ / pHistory_ * c->expectedReward_;
        familySolved = familySolved && c->isSymbolicallySolved_;
      }

      familyStatus( i ) = { familyReward, familySolved };
    }

    // restrieve best family
    double bestReward= m_inf();
    int bestFamilyId = -1;
    for( auto i = 0; i < families_.d0; ++i )
    {
      if( familyStatus( i ).reward >= bestReward )
      {
        bestReward = familyStatus( i ).reward;
        bestFamilyId = i;
      }
    }

    // retrieve best decision id
    bestFamily_ = families_( bestFamilyId );
    uint bestA = bestFamily_.first()->a_;
    expectedReward_ = bestReward; // this one is more informed!
    expectedBestA_ = bestA;
    isSymbolicallySolved_ = familyStatus( bestFamilyId ).solved;

    // check
    //std::cout << familyRewards << std::endl;
    //std::cout << actionStr( bestA ) << std::endl;
    //std::cout << "best family size:" << bestFamily_.d0 << " solved?" << familyStatus( bestFamilyId ).solved << std::endl;
    //
  }

  if( parent_ )
    parent_->backTrackBestExpectedPolicy();
}

void POLGPNode::solvePoseProblem()
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      mlr::KinematicWorld kin = isRoot() ? *startKinematics_( w ) : parent_->effKinematics_( w );

      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( kin );
      komo->setTiming( 1., 2, 5., 1, false );
      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQVelocities();
      komo->setSquaredFixSwitchedObjects(-1., -1., 1e3);

      //std::cout << *folStates_( w ) << std::endl;

      komo->groundTasks( 0., *folStates_( w ) );

      DEBUG( FILE("z.fol") <<fol; )
      DEBUG( komo->MP->reportFeatures( true, FILE( "z.problem" ) ); )
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
      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )

      Graph result = komo->getReport();
      DEBUG( FILE( "z.problem.cost" ) << result; )
      double cost = result.get<double>( { "total","sqrCosts" } );
      double constraints = result.get<double>( { "total","constraints" } );

      if( ! isRoot() )
      {
        cost += parent_->poseProblem_.costs_( w );
      }

      // if this pose leads to the smaller cost so far
      if( ! poseProblem_.solved_( w ) || cost < poseProblem_.costs_( w ) )
      {
        bool solved =  constraints< maxConstraints_ && cost < maxCost_;

//        if( ! solved )
//        {
//          std::cout << "!!!can't be solved for:" << id_ << " cost:" << cost << std::endl;
//        }
//        else
//        {
//          std::cout << "ok for:" << id_ << " cost:" << cost << std::endl;
//        }

        poseProblem_.costs_( w ) = cost;
        poseProblem_.constraints_( w ) = constraints;
        poseProblem_.solved_( w )    = solved;
        poseProblem_.feasibles_( w ) = solved;
        poseProblem_.komos_( w ) = komo;

        // update effective kinematic
        effKinematics_( w ) = *poseProblem_.komos_( w )->MP->configurations.last();

        // update switch
        for( mlr::KinematicSwitch *sw: poseProblem_.komos_( w )->MP->switches )
        {
          //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
          if( sw->timeOfApplication>=2 ) sw->apply( effKinematics_( w ) );
        }
        effKinematics_( w ).topSort();
        DEBUG( effKinematics_( w ).checkConsistency(); )
            effKinematics_( w ).getJointState();
      }

      // inform symbolic level
//      if( ! poseFeasibles_( w ) )
//        labelInfeasible();

    }
  }

  updateAndBacktrackPoseState();
}

void POLGPNode::solveSeqProblem()
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *startKinematics_( w ) );
      komo->setTiming( time_, 2, 5., 1, false );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQVelocities();
      komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath ){
        auto time = ( node->parent_ ? node->parent_->time_: 0. ); // get parent time
        komo->groundTasks( time, *node->folStates_( w ) );        // ground parent action (included in the initial state)
      }

      DEBUG( FILE("z.fol") << folWorlds_( w ); )
          DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
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

      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
          //  komo.checkGradients();

      Graph result = komo->getReport();
      DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! seqProblem_.komos_( w ) || cost < seqProblem_.costs_( w ) )
      {
        bool solved =  constraints< maxConstraints_;

        seqProblem_.costs_( w )       = cost;
        seqProblem_.constraints_( w ) = constraints;
        seqProblem_.solved_( w )      = solved;
        seqProblem_.feasibles_( w )   = solved;
        seqProblem_.komos_( w )       = komo;
      }

//      if( ! seqFeasibles_( w ) )
//        labelInfeasible();
    }
  }

  updateAndBacktrackSequenceState();
}

void POLGPNode::solvePathProblem( uint microSteps )
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *startKinematics_( w ) );
      komo->setTiming( time_, microSteps, 5., 2, false );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQAccelerations();
      komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        auto time = ( node->parent_ ? node->parent_->time_: 0. );   // get parent time
        komo->groundTasks( time, *node->folStates_( w ) );          // ground parent action (included in the initial state)
      }

      DEBUG( FILE("z.fol") <<fol; )
          DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
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

      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      //komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! pathProblem_.costs_( w ) || cost < pathProblem_.costs_( w ) )     //
      {
        bool solved =  constraints< maxConstraints_;

        pathProblem_.costs_( w )       = cost;                     //
        pathProblem_.constraints_( w ) = constraints;              //
        pathProblem_.solved_( w )      = solved;
        pathProblem_.feasibles_( w )   = solved;                   //
        pathProblem_.komos_( w )       = komo;                     //

        // back the best komo for this world
        for( POLGPNode * node = this; node; node = node->parent_ )
        {
          node->pathProblem_.komos_( w ) = komo;
        }
      }

//      if( ! pathFeasibles_( w ) )
//        labelInfeasible();
    }
  }

  updateAndBacktrackPathState();
}

void POLGPNode::solveJointPathProblem( uint microSteps )
{
  //-- collect 'path nodes'
  POLGPNodeL treepath = getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();

      // set-up komo
      komo->setModel( *startKinematics_( w ) );
      komo->setTiming( time_, microSteps, 5., 2, false );

      komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
      komo->setSquaredQAccelerations();
      komo->setSquaredFixJointVelocities( -1., -1., 1e3 );
      komo->setSquaredFixSwitchedObjects( -1., -1., 1e3 );

      for( auto node:treepath )
      {
        // set task
        auto time = ( node->parent_ ? node->parent_->time_: 0. );   // get parent time
        komo->groundTasks( time, *node->folStates_( w ) );          // ground parent action (included in the initial state)

        if( node->time_ > 0 )
        {
          uint pathMicroSteps = ( pathProblem_.komos_( w )->MP->configurations.N - 1 ) / time_;
          uint nodeSlice = pathMicroSteps * node->time_ - 1;
          arr q = zeros( pathProblem_.komos_( w )->MP->configurations( nodeSlice )->q.N );

          // set constraints enforcing the path equality among worlds
          for( auto x = 0; x < N_; ++x )
          {
            if( node->bs_( x ) > 0 )
            {
              auto komo = node->pathProblem_.komos_( x );

              CHECK( node->pathProblem_.komos_( x )->MP->configurations.N > 0, "one node along the solution path doesn't have a path solution already!" );

              q += node->bs_( x ) * node->pathProblem_.komos_( x )->MP->configurations( nodeSlice )->q;
            }
          }

          AgentKinEquality * task = new AgentKinEquality( node->id_, q );  // tmp camille, think to delete it, or komo does it?

          //std::cout << "t:" << node->time_ << " q.N " << q.N  << std::endl;

          komo->setTask( node->time_ - 1.0 / pathMicroSteps, node->time_ - 1.0 / pathMicroSteps, task, OT_sumOfSqr, NoArr, 1e2  );

        }
      }

      DEBUG( FILE("z.fol") <<fol; )
          DEBUG( komo->MP->reportFeatures( true, FILE("z.problem") ); )
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

      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      //komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! jointProblem_.costs_( w ) || cost < jointProblem_.costs_( w ) )       //
      {
        bool solved =  constraints< maxConstraints_;

        jointProblem_.costs_( w )       = cost;                     //
        jointProblem_.constraints_( w ) = constraints;              //
        jointProblem_.solved_( w )      = solved;         //
        jointProblem_.feasibles_( w )   = solved;         //
        jointProblem_.komos_( w )       = komo;
      }
//      if( ! jointPathFeasibles_( w ) )
//        labelInfeasible();
    }
  }

  updateAndBacktrackJointPathState();
}

void POLGPNode::labelInfeasible()
{
  // set flag and badest reward
  isInfeasible_ = true;
  expectedReward_ = m_inf();

  // delete children nodes
//  for( auto children : families_ )
//  {
//    DEL_INFEASIBLE( children.clear(); )
//  }
//  families_.clear();

  // backtrack results
  if( parent_ )
  {
    parent_->backTrackBestExpectedPolicy();
  }
  //-- remove children
//  ActionNodeL tree;
//  getAllChildren(tree);
//  for(ActionNode *n:tree) if(n!=this) delete n; //TODO: memory leak!
//  for( auto children : families_ )
//  {
//    DEL_INFEASIBLE( children.clear(); )
//  }
//  families_.clear();

//  //-- add INFEASIBLE flag to fol
//  auto folDecision = folStates_( w )->getNode("decision");
//  NodeL symbols = folDecision->parents;
//  symbols.prepend( folWorlds_( w )->KB.getNode({"INFEASIBLE"}));

////  cout <<"\n *** LABELLING INFEASIBLE: "; listWrite(symbols); cout <<endl;
//  //-- find the right parent...
//  POLGPNode* node = this;
//  while( node->parent_ ){
//    bool stop=false;
//    for(Node *fact:node->folStates_( w )->list()){
//      if(fact->keys.N && fact->keys.last()=="block"){
//        if(tuplesAreEqual(fact->parents, symbols)){
//          CHECK(fact->isOfType<bool>() && fact->keys.first()=="block", "");
//          stop=true;
//          break;
//        }
//      }
//    }
//    if(stop) break;
//    node = node->parent_;
//  }

//  if( ! node->folAddToStates_( w ) )
//  {
//    node->folAddToStates_( w ) = &folWorlds_( w )->KB.newSubgraph({"ADD"}, {node->folStates_( w )->isNodeOfGraph})->value;
//  }
//  node->folAddToStates_( w )->newNode<bool>({}, symbols, true);

////  ActionNode *root=getRoot();
//  node->recomputeAllFolStates();
//  node->recomputeAllMCStats(false);
}

mlr::Array< POLGPNode * > POLGPNode::getTreePath()
{
  mlr::Array< POLGPNode * > path;
  POLGPNode * node = this;
  for(;node;){
    path.prepend(node);
    node = node->parent_;
  }
  return path;
}

mlr::Array< POLGPNode * > POLGPNode::getTreePathFrom( POLGPNode * start )
{
  mlr::Array< POLGPNode * > subPath;

  POLGPNode * node = this;
  do
  {
    subPath.prepend( node );
    node = node->parent_;

//    if( node == start )
//      std::cout << "node == start " << std::endl;

  } while ( ( node != start ) && node );

  return subPath;
}

void POLGPNode::updateAndBacktrackPoseState()
{
  if( isSymbolicallyTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        solved = solved && poseProblem_.solved_( w );

        if( ! poseProblem_.feasibles_( w ) )
        {
          labelInfeasible(); // label this sequence of actions as infeasible
        }
      }
    }

    poseProblem_.isSolved_ = solved;

    if( poseProblem_.isSolved_ )
    {
      poseProblem_.isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        solved = solved && poseProblem_.solved_( w );

        if( ! poseProblem_.feasibles_( w ) )
        {
          labelInfeasible(); // label this sequence of actions as infeasible
        }
      }
    }

    for( auto s : bestFamily() )
    {
      solved = solved && s->poseProblem_.isSolved_;
    }

    poseProblem_.isSolved_ = solved;
  }

  // continue backtracking
  if( parent_ )
  {
    parent_->updateAndBacktrackPoseState();
  }
}

void POLGPNode::updateAndBacktrackSequenceState()
{
  if( poseProblem_.isTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        solved = solved && seqProblem_.solved_( w );
      }
    }
    seqProblem_.isSolved_ = solved;

    if( seqProblem_.isSolved_ )
    {
      seqProblem_.isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : bestFamily() )
    {
      solved = solved && s->seqProblem_.isSolved_;
    }

    seqProblem_.isSolved_ = solved;
  }

  // continue backtracking
  if( parent_ )
  {
    parent_->updateAndBacktrackSequenceState();
  }
}

void POLGPNode::updateAndBacktrackPathState()
{
  if( seqProblem_.isTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        solved = solved && pathProblem_.solved_( w );
      }
    }
    pathProblem_.isSolved_ = solved;

    if( pathProblem_.isSolved_ )
    {
      pathProblem_.isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : bestFamily() )
    {
      solved = solved && s->pathProblem_.isSolved_;
    }

    pathProblem_.isSolved_ = solved;
  }

  // continue backtracking
  if( parent_ )
  {
    parent_->updateAndBacktrackPathState();
  }
}

void POLGPNode::updateAndBacktrackJointPathState()
{
  if( pathProblem_.isTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        solved = solved && jointProblem_.solved_( w );
      }
    }
    jointProblem_.isSolved_ = solved;

    if( jointProblem_.isSolved_ )
    {
      jointProblem_.isTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : bestFamily() )
    {
      solved = solved && s->jointProblem_.isSolved_;
    }

    jointProblem_.isSolved_ = solved;
  }

  // continue backtracking
  if( parent_ )
  {
    parent_->updateAndBacktrackJointPathState();
  }
}

uint POLGPNode::getPossibleActionsNumber() const
{
  auto logicAndState = getWitnessLogicAndState();

  logicAndState.logic->setState( logicAndState.state.get(), d_ );

  auto actions = logicAndState.logic->get_actions();

  return actions.size();
}

LogicAndState POLGPNode::getWitnessLogicAndState() const
{
  auto worlds = getPossibleLogicAndStates();

  CHECK( worlds.d0 > 0, "Unable to find a witness logic!!" );

  return worlds.first();
}

mlr::Array< LogicAndState > POLGPNode::getPossibleLogicAndStates() const
{
  mlr::Array< LogicAndState > worlds;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      worlds.append( { folWorlds_( w ), folStates_( w ) } );
    }
  }

  return worlds;
}

std::string POLGPNode::actionStr( uint a ) const
{
  auto ls = getWitnessLogicAndState();
  ls.logic->reset_state();
  ls.logic->setState( ls.state.get() );
  auto actions = ls.logic->get_actions();

  std::stringstream ss;

  if( a >= 0 && a < actions.size() )
    ss << *actions[a];
  else
    ss << "no actions";

  return ss.str();
}

//===========================================================================

RUN_ON_INIT_BEGIN(manipulationTree)
POLGPNodeL::memMove = true;
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
