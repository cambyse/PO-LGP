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


#include "ao_node.h"

#include <unordered_map>

#include <list>

#include <boost/algorithm/string/replace.hpp>

#include <MCTS/solver_PlainMC.h>

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) x

//=====================free functions======================
static double eps() { return std::numeric_limits< double >::epsilon(); }

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

struct AgentKinEquality:TaskMap{

  AgentKinEquality( const arr& q/*, const arr& qdot*/ )
    : q_  ( q )
    , dim_( q.N )
  {

  }

  virtual void phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1 )
  {
    y = zeros( dim_ );
    y.setVectorBlock( G.q - q_, 0 );

    auto tmp_J = eye( dim_, dim_ );

    if(&J) J = tmp_J;
  }

  virtual uint dim_phi( const mlr::KinematicWorld& G )
  {
    return dim_;
  }

  virtual mlr::String shortTag( const mlr::KinematicWorld& G )
  {
    return mlr::String( "AgentKinEquality" );
  }

private:
  // parameters
  const arr q_;
  const uint dim_;
};

//===========================================================================

static int nodeNumber = 0;

/// root node init
AONode::AONode( mlr::Array< std::shared_ptr< FOL_World > > fols, const mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > & kins, const arr & bs, const KOMOFactory & komoFactory )
  : parent_( nullptr )
  , N_( fols.N )
  , folWorlds_( fols )
  , folStates_( N_ )
  , folAddToStates_( N_ )
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
  , isTerminal_( false )
  , isSolved_( false )
  // logic search
  , isSymbolicallyTerminal_( false )
  , isSymbolicallySolved_( false )
  , rootMCs_( N_ )
  , mcStats_( new MCStatistics )
  , expectedReward_( 0 )
  , expectedBestA_( -1 )
  , komoFactory_( komoFactory )
  // poseOpt
  , poseCosts_       ( N_ )
  , poseConstraints_ ( N_ )
  , poseSolved_      ( N_ )
  , poseFeasibles_   ( N_ )
  , komoPoseProblems_( N_ )
  , isPoseTerminal_     ( false )
  , isPoseProblemSolved_( false )
  // seqOpt
  , seqCosts_        ( N_ )
  , seqConstraints_  ( N_ )
  , seqSolved_       ( N_ )
  , seqFeasibles_    ( N_ )
  , komoSeqProblems_ ( N_ )
  , isSequenceTerminal_     ( false )
  , isSequenceProblemSolved_( false )
  // pathOpt
  , pathCosts_       ( N_ )
  , pathConstraints_ ( N_ )
  , pathFeasibles_   ( N_ )
  , komoPathProblems_( N_ )
  , isPathTerminal_     ( false )
  , isPathProblemSolved_( false )
  // jointPathOpt
  , jointPathCosts_      ( N_ )
  , jointPathConstraints_( N_ )
  , jointPathFeasibles_  ( N_ )
  , komoJointPathProblems_( N_ )
  , isJointPathTerminal_     ( false )
  , isJointPathProblemSolved_( false )
  //
  , id_( 0 )
{
  for( auto w = 0; w < N_; ++w )
  {
    folWorlds_( w )->reset_state();
    folStates_( w ).reset( folWorlds_( w )->createStateCopy() );
    folAddToStates_( w ) = nullptr;
    rootMCs_( w ).reset( new PlainMC( *folWorlds_( w ) ) );
    rootMCs_( w )->verbose = 0;

    // pose
    poseCosts_( w ) = 0;
    poseConstraints_( w ) = 0;
    poseSolved_( w ) = false;
    poseFeasibles_( w ) = true;

    // seq
    seqCosts_( w ) = 0;
    seqConstraints_( w ) = 0;
    seqSolved_( w ) = false;
    seqFeasibles_( w ) = true;
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
AONode::AONode(AONode *parent, double pHistory, const arr & bs, uint a )
  : parent_( parent )
  , N_( parent_->N_ )
  , folWorlds_( parent->folWorlds_ )
  , folStates_( N_ )
  , folAddToStates_( N_ )
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
  , isTerminal_( false )
  , isSolved_( false )
  // logic search
  , isSymbolicallyTerminal_( false )
  , isSymbolicallySolved_( false )
  , rootMCs_( parent->rootMCs_ )
  , mcStats_( new MCStatistics )
  , expectedReward_( 0 )
  , expectedBestA_ (-1 )
  , komoFactory_( parent->komoFactory_ )
  // poseOpt
  , poseCosts_       ( N_ )
  , poseConstraints_ ( N_ )
  , poseSolved_      ( N_ )
  , poseFeasibles_   ( N_ )
  , komoPoseProblems_( N_ )
  , isPoseProblemSolved_( false )
  // seqOpt
  , seqCosts_        ( N_ )
  , seqConstraints_  ( N_ )
  , seqSolved_       ( N_ )
  , seqFeasibles_    ( N_ )
  , komoSeqProblems_ ( N_ )
  , isSequenceTerminal_  ( false )
  // pathOpt
  , pathCosts_       ( N_ )
  , pathConstraints_ ( N_ )
  , pathFeasibles_   ( N_ )
  , komoPathProblems_( N_ )
  , isPathTerminal_  ( false )
  // jointPathOpt
  , jointPathCosts_  ( N_ )
  , jointPathConstraints_( N_ )
  , jointPathFeasibles_( N_ )
  , komoJointPathProblems_( N_ )
  , isJointPathTerminal_( false )
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

      folAddToStates_( w ) = nullptr;

      decisions_( w ) = actions[ a_ ];

      // pose
      poseCosts_( w ) = 0;
      poseConstraints_( w ) = 0;
      poseSolved_( w ) = false;
      poseFeasibles_( w ) = true;

      // seq
      seqCosts_( w ) = 0;
      seqConstraints_( w ) = 0;
      seqSolved_( w )    = false;
      seqFeasibles_( w ) = true;
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

void AONode::expand()
{
  // debug
  //std::cout << std::endl;
  //std::cout << "AONode::expand().." << std::endl;
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
    mlr::Array< AONode * > familiy;
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

      auto n = new AONode( this, pWorld * pHistory_, bs, a );
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

void AONode::setAndSiblings( const mlr::Array< AONode * > & siblings )
{
  for( auto s : siblings )
  {
    if( s != this )
    andSiblings_.append( s );
  }
}

void AONode::generateMCRollouts( uint num, int stepAbort )
{
  //std::cout << "AONode::generateMCRollouts.." << std::endl;
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

void AONode::backTrackBestExpectedPolicy()
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
    double bestReward= -std::numeric_limits< double >::max();
    uint bestFamilyId = -1;
    for( auto i = 0; i < families_.d0; ++i )
    {
      if( familyStatus( i ).reward > bestReward )
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
    bestFamily_ = families_( bestFamilyId );
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

void AONode::solvePoseProblem()
{
  //-- collect 'path nodes'
  AONodeL treepath = getTreePath();

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
      //auto komo = getWitnessPoseKomo();
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

      if( ! isRoot() ) cost += parent_->poseCosts_( w );

      // if this pose leads to the smaller cost so far
      if( ! poseSolved_( w ) || cost < poseCosts_( w ) )
      {
        poseCosts_( w ) = cost;
        poseConstraints_( w ) = constraints;
        poseSolved_( w )    = ( constraints< .5 ); // tmp camille
        poseFeasibles_( w ) = ( constraints< .5 ); // tmp camille
        komoPoseProblems_( w ) = komo;

        // update effective kinematic
        effKinematics_( w ) = *komoPoseProblems_( w )->MP->configurations.last();

        // update switch
        for( mlr::KinematicSwitch *sw: komoPoseProblems_( w )->MP->switches )
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

  if( isSymbolicallyTerminal() )
    updateAndBacktrackPoseState();
}

void AONode::solveSeqProblem()
{
  //-- collect 'path nodes'
  AONodeL treepath = getTreePath();

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
        komo->groundTasks( ( node->parent_ ? node->parent_->time_:0. ), *node->folStates_( w ) );//groundTasks((node->parent?node->parent->time:0.), *node->folState);
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

      if( ! komoSeqProblems_( w ) || cost < seqCosts_( w ) )
      {
        seqCosts_( w ) = cost;
        seqConstraints_( w ) = constraints;
        seqSolved_( w )    = (constraints<.5);
        seqFeasibles_( w ) = (constraints<.5);
        komoSeqProblems_( w ) = komo;
      }

//      if( ! seqFeasibles_( w ) )
//        labelInfeasible();
    }
  }

  updateAndBacktrackSequenceState();
}

void AONode::solvePathProblem( uint microSteps )
{
  //-- collect 'path nodes'
  AONodeL treepath = getTreePath();

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
        komo->groundTasks( ( node->parent_?node->parent_->time_: 0. ), *node->folStates_( w ) );
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

      if( ! komoPathProblems_( w ) || cost < pathCosts_( w ) )     //
      {                                                   //
        pathCosts_( w )       = cost;                     //
        pathConstraints_( w ) = constraints;              //
        pathFeasibles_( w )   = (constraints<.5);         //
        komoPathProblems_( w ) = komo;                    //

        // back the best komo for this world
        for( AONode * node = this ;node; node = node->parent_ )
        {
          node->komoPathProblems_( w ) = komo;
        }
      }

//      if( ! pathFeasibles_( w ) )
//        labelInfeasible();
    }
  }

  updateAndBacktrackPathState();
}

void AONode::solveJointPathProblem( uint microSteps )
{  
  //-- collect 'path nodes'
  AONodeL treepath = getTreePath();

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
        auto time = ( node->parent_ ? node->parent_->time_: 0. );
        komo->groundTasks( ( node->parent_ ? node->parent_->time_: 0. ), *node->folStates_( w ) );

        uint pathMicroSteps = ( komoPathProblems_( w )->MP->configurations.N - 1 ) / time_;
        uint nodeSlice = pathMicroSteps * node->time_;
        arr q = zeros( komoPathProblems_( w )->MP->configurations.first()->q.N );

        // set constraints enforcing the path equality among worlds
        for( auto x = 0; x < N_; ++x )
        { 
          if( node->bs_( x ) > 0 )
          {
            CHECK( node->komoPathProblems_( x )->MP->configurations.N > 0, "one node along the solution path doesn't have a path solution already!" );

            q += node->bs_( x ) * node->komoPathProblems_( x )->MP->configurations( nodeSlice )->q;
          }
        }

        AgentKinEquality * task = new AgentKinEquality( q );  // tmp camille, think to delete it, or komo does it?

        komo->setTask( node->time_ - 1.0 / pathMicroSteps, node->time_, task );
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

      if( ! komoJointPathProblems_( w ) || cost < jointPathCosts_( w ) )       //
      {                                                   //
        jointPathCosts_( w )       = cost;                     //
        jointPathConstraints_( w ) = constraints;              //
        jointPathFeasibles_( w )   = (constraints<.5);         //
        komoJointPathProblems_( w )= komo;
      }
//      if( ! jointPathFeasibles_( w ) )
//        labelInfeasible();
    }
  }

  updateAndBacktrackJointPathState();
}

void AONode::labelInfeasible( uint w )
{
  //-- remove children
//  ActionNodeL tree;
//  getAllChildren(tree);
//  for(ActionNode *n:tree) if(n!=this) delete n; //TODO: memory leak!
  for( auto children : families_ )
  {
    DEL_INFEASIBLE( children.clear(); )
  }

  //-- add INFEASIBLE flag to fol
  auto folDecision = folStates_( w )->getNode("decision");
  NodeL symbols = folDecision->parents;
  symbols.prepend( folWorlds_( w )->KB.getNode({"INFEASIBLE"}));

//  cout <<"\n *** LABELLING INFEASIBLE: "; listWrite(symbols); cout <<endl;
  //-- find the right parent...
  AONode* node = this;
  while( node->parent_ ){
    bool stop=false;
    for(Node *fact:node->folStates_( w )->list()){
      if(fact->keys.N && fact->keys.last()=="block"){
        if(tuplesAreEqual(fact->parents, symbols)){
          CHECK(fact->isOfType<bool>() && fact->keys.first()=="block", "");
          stop=true;
          break;
        }
      }
    }
    if(stop) break;
    node = node->parent_;
  }

//  if(!node->folAddToState){
//    node->folAddToState = &fol.KB.newSubgraph({"ADD"}, {node->folState->isNodeOfGraph})->value;
//  }
//  node->folAddToState->newNode<bool>({}, symbols, true);

////  ActionNode *root=getRoot();
//  node->recomputeAllFolStates();
//  node->recomputeAllMCStats(false);
}

mlr::Array< AONode * > AONode::getTreePath()
{
  mlr::Array< AONode * > path;
  AONode * node = this;
  for(;node;){
    path.prepend(node);
    node = node->parent_;
  }
  return path;
}

mlr::Array< AONode * > AONode::getTreePathFrom( AONode * start )
{
  mlr::Array< AONode * > subPath;

  AONode * node = this;
  do
  {
    subPath.prepend( node );
    node = node->parent_;

//    if( node == start )
//      std::cout << "node == start " << std::endl;

  } while ( ( node != start ) && node );

  return subPath;
}

void AONode::updateAndBacktrackPoseState()
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
        solved = solved && poseSolved_( w );

        if( ! poseFeasibles_( w ) )
        {
          labelInfeasible( w ); // label this sequence of actions as infeasible for this world
        }
      }
    }
    isPoseProblemSolved_ = solved;

    if( isPoseProblemSolved_ )
    {
      isPoseTerminal_      = true;
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
        solved = solved && poseSolved_( w );

        if( ! poseFeasibles_( w ) )
        {
          labelInfeasible( w ); // label this sequence of actions as infeasible for this world
        }
      }
    }

    for( auto s : bestFamily() )
    {
      solved = solved && s->isPoseProblemSolved_;
    }

    isPoseProblemSolved_ = solved;
  }

  // continue backtracking
  if( parent_ )
  {
    parent_->updateAndBacktrackPoseState();
  }
}

void AONode::updateAndBacktrackSequenceState()
{
  if( isPoseTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        solved = solved && seqSolved_( w );
      }
    }
    isSequenceProblemSolved_ = solved;

    if( isSequenceProblemSolved_ )
    {
      isSequenceTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : bestFamily() )
    {
      solved = solved && s->isSequenceProblemSolved_;
    }

    isSequenceProblemSolved_ = solved;
  }

  // continue backtracking
  if( parent_ )
  {
    parent_->updateAndBacktrackSequenceState();
  }
}

void AONode::updateAndBacktrackPathState()
{
  if( isSequenceTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        solved = solved && pathFeasibles_( w );
      }
    }
    isPathProblemSolved_ = solved;

    if( isPathProblemSolved_ )
    {
      isPathTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : bestFamily() )
    {
      solved = solved && s->isPathProblemSolved_;
    }

    isPathProblemSolved_ = solved;
  }

  // continue backtracking
  if( parent_ )
  {
    parent_->updateAndBacktrackPathState();
  }
}

void AONode::updateAndBacktrackJointPathState()
{
  if( isPathTerminal_ )
  {
    // if the node is logically terminal and if a pose has been found for each world,
    // then the node is considered as pose-solve and pose-terminal

    bool solved = true;
    // test if it is solved for all worlds
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        solved = solved && jointPathFeasibles_( w );
      }
    }
    isJointPathProblemSolved_ = solved;

    if( isJointPathProblemSolved_ )
    {
      isJointPathTerminal_      = true;
    }
  }
  else
  {
    // if the node is not terminal, we set it as solve, if the pose optimization was successful and the children of its best family are solved
    bool solved = true;

    for( auto s : bestFamily() )
    {
      solved = solved && s->isJointPathProblemSolved_;
    }

    isJointPathProblemSolved_ = solved;
  }

  // continue backtracking
  if( parent_ )
  {
    parent_->updateAndBacktrackJointPathState();
  }
}

uint AONode::getPossibleActionsNumber() const
{
  auto logicAndState = getWitnessLogicAndState();

  logicAndState.logic->setState( logicAndState.state.get(), d_ );

  auto actions = logicAndState.logic->get_actions();

  return actions.size();
}

LogicAndState AONode::getWitnessLogicAndState() const
{
  auto worlds = getPossibleLogicAndStates();

  CHECK( worlds.d0 > 0, "Unable to find a witness logic!!" );

  return worlds.first();
}

mlr::Array< LogicAndState > AONode::getPossibleLogicAndStates() const
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

std::string AONode::actionStr( uint a ) const
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
AONodeL::memMove = true;
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
