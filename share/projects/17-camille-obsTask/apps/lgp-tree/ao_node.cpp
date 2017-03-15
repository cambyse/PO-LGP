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
//  std::string retString;
//  for( auto s : facts )
//    retString += s;

//  return retString;
}

struct AgentKinEquality:TaskMap{

  AgentKinEquality( const arr& q/*, const arr& qdot*/ )
    : q_  ( q )
    //, qdot_( qdot_ )
    , dim_( q.N )
  {

  }

  virtual void phi( arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1 )
  {
    y = zeros( dim_ );
    y.setVectorBlock( G.q - q_, 0 );
    //y.setVectorBlock( G.qdot - qdot_, q_.N - 1 );

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
  //const arr& qdot_;
  const uint dim_;
};

//===========================================================================

static int nodeNumber = 0;

/// root node init
AONode::AONode( mlr::Array< std::shared_ptr< FOL_World > > fols, const mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > & kins, const arr & bs, const KOMOFactory & komoFactory )
  : parent_( nullptr )
  , folWorlds_( fols )
  , folStates_( folWorlds_.d0 )
  , startKinematics_( kins )
  , effKinematics_( folWorlds_.d0 )
  , effKinematicsPaths2_( folWorlds_.d0 )
  , effKinematicsPaths2areSet_( folWorlds_.d0 )
  , pHistory_( 1.0 )
  , bs_( bs )
  , a_( -1 )
  , d_( 0 )
  , time_( 0.0 )
  , isExpanded_( false )
  , isTerminal_( false )
  , isSymbolicallySolved_( false )
  , isInfeasible_( false )
  , rootMCs_( bs_.d0 )
  , mcStats_( new MCStatistics )
  , expectedReward_( 0 )
  , expectedBestA_( -1 )
  , komoFactory_( komoFactory )
  // poseOpt
  , poseCost_( 0.0 )
  , poseConstraints_( 0.0 )
  , poseFeasible_( false )
  , komoPoseProblems_( bs_.d0 )
  // seqOpt
  , seqCost_( 0.0 )
  , seqConstraints_( 0.0 )
  , seqFeasible_( false )
  , komoSeqProblems_ ( bs_.d0 )
  // pathOpt
  , pathCosts_       ( bs_.d0 )
  , pathConstraints_ ( bs_.d0 )
  , pathFeasibles_   ( bs_.d0 )
  , paths_           ( bs_.d0 )
  , komoPathProblems_( bs_.d0 )
  // jointPathOpt
  , jointPathCost_( 0.0 )
  , jointPathConstraints_( 0.0 )
  , jointPathFeasible_( false )
  , komoJointPathProblems_( bs_.d0 )
  //
  , id_( 0 )
{
  for( auto w = 0; w < bs_.d0; ++w )
  {
    folWorlds_( w )->reset_state();
    folStates_( w ).reset( folWorlds_( w )->createStateCopy() );
    rootMCs_( w ).reset( new PlainMC( *folWorlds_( w ) ) );
    rootMCs_( w )->verbose = 0;
  }

  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    effKinematics_( w ) = mlr::KinematicWorld( * startKinematics_( w ) );
    effKinematicsPaths2areSet_( w ) = false;
  }

  std::size_t s = 0;

  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      s++;
    }
  }

  supportSize_ = s;
}

/// child node creation
AONode::AONode(AONode *parent, double pHistory, const arr & bs, uint a )
  : parent_( parent )
  , folWorlds_( parent->folWorlds_ )
  , folStates_( folWorlds_.d0 )
  , startKinematics_( parent->startKinematics_ )
  , effKinematics_( parent->effKinematics_ )
  , effKinematicsPaths2_( parent->effKinematicsPaths2_ )
  , effKinematicsPaths2areSet_( parent->effKinematicsPaths2areSet_.d0 )
  , decisions_( folWorlds_.d0 )
  , pHistory_( pHistory )
  , bs_( bs )
  , a_( a )
  , d_( parent->d_ + 1 )
  , isExpanded_( false )
  , isTerminal_( false )
  , isSymbolicallySolved_( false )
  , isInfeasible_( false )
  , rootMCs_( parent->rootMCs_ )
  , mcStats_( new MCStatistics )
  , expectedReward_( 0 )
  , expectedBestA_( -1 )
  , komoFactory_( parent->komoFactory_ )
  // poseOpt
  , poseCost_( 0.0 )
  , poseConstraints_( 0.0 )
  , poseFeasible_( false )
  , komoPoseProblems_( parent->bs_.d0   )
  // seqOpt
  , seqCost_( 0.0 )
  , seqConstraints_( 0.0 )
  , seqFeasible_( false )
  , komoSeqProblems_ ( parent->bs_.d0   )
  // pathOpt
  , pathCosts_       ( parent->bs_.d0   )
  , pathConstraints_ ( parent->bs_.d0   )
  , pathFeasibles_   ( parent->bs_.d0   )
  , paths_           ( parent->bs_.d0   )
  , komoPathProblems_( parent->bs_.d0   )
  // jointPathOpt
  , jointPathCost_( 0.0 )
  , jointPathConstraints_( 0.0 )
  , jointPathFeasible_( false )
  , komoJointPathProblems_( parent->bs_.d0   )
  //, path2Configurations_( parent->bs_.d0   )
{
  // update the states
  bool isTerminal = true;
  for( auto w = 0; w < folWorlds_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
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

      decisions_( w ) = actions[ a_ ];
    }
  }
  isTerminal_ = isTerminal;

  if( isTerminal )
    isSymbolicallySolved_ = true;

//  if( isTerminal_ )
//  {
//    std::cout << "found terminal node!" << bs_ << std::endl;
//  }

  for( auto w = 0; w < effKinematicsPaths2areSet_.d0; ++w )
  {
    effKinematicsPaths2areSet_( w ) = false;
  }

  // update time
  auto ls = getWitnessLogicAndState();
  time_ = parent_->time_ + ls.logic->lastStepDuration;

  // update support size
  std::size_t s = 0;

  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      s++;
    }
  }

  supportSize_ = s;

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
  if( isTerminal_ )
    return;

  // get possible actions for the worlds having a non null probability
  auto nActions = getPossibleActionsNumber();

  //std::cout << "number of possible actions:" << nActions << std::endl;

  if( nActions == 0 )
    isTerminal_ = true;

  for( auto a = 0; a < nActions; ++a )
  {
    //std::cout << "------------" << std::endl;
    //std::cout << "action:" << a << std::endl;
    stringSetHash f;
    std::unordered_map< std::set< std::string >, std::list< uint >, stringSetHash > outcomesToWorlds;

    for( auto w = 0; w < folWorlds_.d0; ++w )
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
      arr bs = zeros( bs_.d0 );
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

  for( auto w = 0; w < folWorlds_.d0; ++w )
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
  if( isTerminal() )
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
  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      mlr::KinematicWorld kin = isRoot() ? *startKinematics_( w ) : parent_->effKinematics_( w );

      // create komo
      auto komo = komoFactory_.createKomo();
      komoPoseProblems_( w ) = komo;

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
    }
  }

  // check if all worlds lead to same agent sequence of positions
//  bool sameTrajectories = sameAgentTrajectories( komoPoseProblems_ );

//  if( ! sameTrajectories )
//    labelInfeasible();

  // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
  auto komo = getWitnessPoseKomo();
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

  if( ! isRoot() ) cost += parent_->poseCost_;

  // if this pose leads to the smaller cost so far
  if( ! pose_.N || cost < poseCost_ )
  {
    poseCost_ = cost;
    poseConstraints_ = constraints;
    poseFeasible_ = ( constraints< .5 ); // tmp camille
    pose_ = komo->x;
  }

  // inform symbolic level
  if( ! poseFeasible_ )
    labelInfeasible();

  // update effective kinematic
  for( auto w = 0; w < effKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      effKinematics_( w ) = *komoPoseProblems_( w )->MP->configurations.last();
      //effKinematics_( w ).watch();
    }
  }

  // update switch
  for( auto w = 0; w < effKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      for( mlr::KinematicSwitch *sw: komoPoseProblems_( w )->MP->switches )
      {
        //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
        if( sw->timeOfApplication>=2 ) sw->apply( effKinematics_( w ) );
      }
      effKinematics_( w ).topSort();
      DEBUG( effKinematics_( w ).checkConsistency(); )
        effKinematics_( w ).getJointState();
    }
  }
}

void AONode::solveSeqProblem()
{
  //-- collect 'path nodes'
  AONodeL treepath = getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();
      komoSeqProblems_( w ) = komo;

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
    }
  }

  // check if all worlds lead to same agent sequence of positions
//  bool sameTrajectories = sameAgentTrajectories( komoSeqProblems_ );

//  if( ! sameTrajectories )
//    labelInfeasible();

  // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
  auto komo = getWitnessSeqKomo();
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

  if( ! seq_.N || cost < seqCost_ )
  {
    seqCost_ = cost;
    seqConstraints_ = constraints;
    seqFeasible_ = (constraints<.5);
    seq_ = komo->x;
  }

  if( ! seqFeasible_)
    labelInfeasible();
}

void AONode::solvePathProblem( uint microSteps )
{
  //-- collect 'path nodes'
  AONodeL treepath = getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();
      komoPathProblems_( w ) = komo;

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

      if( ! paths_( w ).N || cost < pathCosts_( w ) )     //
      {                                                   //
        pathCosts_( w )       = cost;                     //
        pathConstraints_( w ) = constraints;              //
        pathFeasibles_( w )   = (constraints<.5);         //
        paths_( w )           = komo->x;                  //

        // back the best komo for this world
        for( AONode * node = this ;node; node = node->parent_ )
        {
          node->komoPathProblems_( w ) = komo;
        }
      }

      if( ! pathFeasibles_( w ) )
        labelInfeasible();
    }
  }
}

void AONode::solveJointPathProblem( uint microSteps )
{  
  //-- collect 'path nodes'
  AONodeL treepath = getTreePath();

  // solve problem for all ( relevant ) worlds
  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      // create komo
      auto komo = komoFactory_.createKomo();
      komoJointPathProblems_( w ) = komo;

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
        for( auto x = 0; x < bs_.d0; ++x )
        { 
          if( node->bs_( x ) > 0 )
          {
            CHECK( node->komoPathProblems_( x )->MP->configurations.N > 0, "one node along the solution path doesn't have a path solution already!" );

            q += node->bs_( x ) * node->komoPathProblems_( x )->MP->configurations( nodeSlice )->q;
          }
        }

        AgentKinEquality * task = new AgentKinEquality( q );  // tmp camille, think to delete it, or komo does it?

        komo->setTask( node->time_, node->time_ + 1.0 / pathMicroSteps, task );
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
      //COUNT_jointPathOpt++;

      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      //komo->checkGradients();

      Graph result = komo->getReport();
      //DEBUG( FILE("z.problem.cost") << result; )
      double cost = result.get<double>({"total","sqrCosts"});
      double constraints = result.get<double>({"total","constraints"});

      if( ! jointPath_.N || cost < jointPathCost_ )     //
      {                                                   //
        jointPathCost_        = cost;                     //
        jointPathConstraints_ = constraints;              //
        jointPathFeasible_    = (constraints<.5);         //
        jointPath_            = komo->x;                  //
      }                                                   //

      if( ! jointPathFeasible_ )
        labelInfeasible();
    }
  }
}

/*void AONode::solvePathProblem2( uint microSteps, AONode * start )
{
  //-- collect 'path nodes'
  AONodeL treepath = getTreePathFrom( start );

  std::cout << "from:" << start->id_ << " to:" << id_ << std::endl;

  // build a kinematic world onto  which to optimize
  auto kin = buildStartOptiKinematic( start );

  // create komo
  auto komo = komoFactory_.createKomo();
  komoPathProblems2_ = komo; // we keep a shared pointer on komo, if not, the kin worlds copied from komo become invalid once the komo has been destroyed, BUG ?

  // set-up komo
  komo->setModel( kin );
  komo->setTiming( time_ - start->time_, microSteps, 5., 2, false );
  komo->setHoming( -1., -1., 1e-1 ); //gradient bug??
  komo->setSquaredQAccelerations();
  komo->setSquaredFixSwitchedObjects(-1., -1., 1e3);

  //std::cout << "---" << std::endl;

  for( auto node:treepath ){
    double t = (node->parent_?node->parent_->time_ - start->time_:0.);
    //std::cout << "t:" << t << std::endl;
    auto ls = node->getWitnessLogicAndState();
    komo->groundTasks( t, *ls.state );
  }
  DEBUG( FILE("z.fol") <<fol; )
      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      komo->reset();
  try{
    komo->run();
  } catch(const char* msg){
    cout << "KOMO FAILED: " << msg <<endl;
  }

  // apply trajectory on all other kin
  //komo->MP->configurations.last()->watch();
  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      path2Configurations_( w ).append( start->path2Configurations_( w ) );           // append old configurations
      //path2Configurations_( w ).append( komo->MP->configurations );

      mlr::Array< mlr::KinematicWorld* > configurations( komo->MP->configurations.N );
      for( auto s = 0; s < komo->MP->configurations.N; ++s )
      {
        auto witness = start->isRoot() ? startKinematics_( w ).get() : start->path2Configurations_( w ).last();
        configurations( s ) = revertToRealKinematic( w, komo->MP->configurations( s ) );
      }
      path2Configurations_( w ).append( configurations );
    }
  }

  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      //path2Configurations_( w ).append( start->path2Configurations_( w ) );           //
      //path2Configurations_( w ).append( komo->MP->configurations.sub( 1, -1 ) );      //
    }
  }

  // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
  COUNT_evals += komo->opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_pathOpt++;

  DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
  //komo->checkGradients();

  Graph result = komo->getReport();
  DEBUG( FILE("z.problem.cost") << result; )
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if( ! path2_.N || cost < pathCost2_ )       //
  {                                           //
    pathCost2_ = cost;                        //
    pathConstraints2_ = constraints;          //
    pathFeasible2_ = (constraints<.5);        //
  }                                           //

  // inform symbolic level
  if( ! pathFeasible2_ )
    labelInfeasible();

  // update effective kinematic
  for( auto w = 0; w < effKinematicsPaths2_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      effKinematicsPaths2_( w ) = *komo->MP->configurations.last();
      effKinematicsPaths2areSet_( w ) = true;
    }
  }

  // update switch
  for( auto w = 0; w < effKinematicsPaths2_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      for( mlr::KinematicSwitch *sw: komo->MP->switches )
      {
        //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
        if( sw->timeOfApplication>=2 ) sw->apply( effKinematicsPaths2_( w ) );
      }
      effKinematicsPaths2_( w ).topSort();
      DEBUG( effKinematicsPaths2_( w ).checkConsistency(); )
        effKinematicsPaths2_( w ).getJointState();
    }
  }
}*/

void AONode::labelInfeasible()
{
  // how to backtrack?
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

  for( auto w = 0; w < folWorlds_.d0; ++w )
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

struct KinFact
{
  std::string name;
  std::string fact;
};

bool operator< ( const KinFact & lhs, const KinFact & rhs )
{
  auto hlhs = std::hash<std::string>()( lhs.name ) ^ std::hash<std::string>()( lhs.fact );
  auto hrhs = std::hash<std::string>()( rhs.name ) ^ std::hash<std::string>()( rhs.fact );

  return hlhs < hrhs;
}

std::set< KinFact > getKinFactIntersection( const std::vector< std::set< KinFact > > & factList )
{
  std::set< KinFact > intersection;
  bool firstInterSet = false;
  for( auto facts : factList )
  {
    if( facts.size() > 0 )
    {
      if( ! firstInterSet )
      {
        intersection = facts;
        firstInterSet = true;
      }
      else
      {
        std::set< KinFact > inter;
        std::set_intersection( intersection.begin(), intersection.end(),
                               facts.begin(), facts.end(),
                               std::inserter( inter, inter.begin() ) );
        intersection = inter;
      }
    }
  }

  return intersection;
}

std::set< KinFact > getDifferentiatingFacts( const std::set< KinFact > & intersection, const std::set< KinFact > & facts )
{
  std::set< KinFact > differenciatingFacts;
  std::set_difference( facts.begin(), facts.end(), intersection.begin(), intersection.end(),
                       std::inserter( differenciatingFacts, differenciatingFacts.begin() ) );

  return differenciatingFacts;
}

/*mlr::KinematicWorld AONode::buildStartOptiKinematic( AONode * start ) const
{
  // get carriage return symbol
  std::stringstream _ss; _ss << std::endl;
  const char carr = *_ss.str().c_str();

  // gathers the list of facts for each world
  std::vector< std::set< KinFact > > bodiesList( startKinematics_.d0 );
  std::vector< std::set< KinFact > > shapesList( startKinematics_.d0 );
  std::vector< std::set< KinFact > > jointsList( startKinematics_.d0 );
  std::vector< std::map< std::string, KinFact > > bodiesMap( startKinematics_.d0 );
  std::vector< std::map< std::string, KinFact > > shapesMap( startKinematics_.d0 );

  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      auto kin = start->isRoot() ? *startKinematics_( w ) : start->effKinematicsPaths2_( w );

      std::set< KinFact > bodiesFacts;
      std::map< std::string, KinFact > bodiesFactsMap;
      for( auto b: kin.bodies )
      {
        std::stringstream name;
        std::stringstream fact;

        name << b->name;

        if (b->name.N) fact <<"body " <<b->name <<" { ";
        b->write(fact);  fact <<" }\n";

        KinFact kf( { name.str(), fact.str() } );
        bodiesFacts.insert( kf );
        bodiesFactsMap[ kf.name ] = kf;
      }
      bodiesList[ w ] = bodiesFacts;
      bodiesMap[ w ]  = bodiesFactsMap;

      std::set< KinFact > shapesFacts;
      std::map< std::string, KinFact > shapesFactsMap;
      for( auto s: kin.shapes )
      {
        std::stringstream name;
        std::stringstream fact;

        name << s->name;

        fact <<"shape ";
        if(s->name.N) fact <<s->name <<' ';
        fact <<"(" <<(s->body?(char*)s->body->name:"") <<"){ ";
        s->write(fact);  fact <<" }\n";

        KinFact kf( { name.str(), fact.str() } );
        shapesFacts.insert(kf );
        shapesFactsMap[ kf.name ] = kf;
      }
      shapesList[ w ] =  shapesFacts;
      shapesMap[ w ]  = shapesFactsMap;

      std::set< KinFact > jointsFacts;
      for( auto j: kin.joints )
      {
        std::stringstream name;
        std::stringstream fact;

        name << j->name;

        fact <<"joint ";
        if (j->name.N) fact <<j->name <<' ';
        fact <<"(" <<j->from->name <<' ' <<j->to->name <<"){ ";
        j->write(fact);  fact <<" }\n";

        jointsFacts.insert( { name.str(), fact.str() } );
      }
      jointsList[ w ] = jointsFacts;
    }
  }

  // compute the fact intersection
  std::set< KinFact > bodiesFactsIntersection = getKinFactIntersection( bodiesList );
  std::set< KinFact > shapesFactsIntersection = getKinFactIntersection( shapesList );
  std::set< KinFact > jointsFactsIntersection = getKinFactIntersection( jointsList );

  // build world with intersection
  std::stringstream ss;
  for( auto fact : bodiesFactsIntersection )
  {
    ss << fact.fact << std::endl;
  }

  for( auto fact : shapesFactsIntersection )
  {
    ss << fact.fact << std::endl;
  }

  for( auto fact : jointsFactsIntersection )
  {
    ss << fact.fact << std::endl;
  }

  // add each fact not in the intersection as an obstacle
  std::string obstaclePrefix = "__obstacle_";
  int obstacleId = 0;

  for( auto w = 0; w < bodiesList.size(); ++w )
  {
    if( bs_( w ) > eps() )
    {
      std::set< KinFact > differentiatingBodiesFacts = getDifferentiatingFacts( bodiesFactsIntersection, bodiesList[ w ] );
      for( auto bodyFact : differentiatingBodiesFacts  )
      {
        // get the corresponding shape fact
        auto shapeFact = shapesMap[ w ][ bodyFact.name ];

        // get the corresponding joints facts
        std::list < KinFact > jointsFacts;
        for( auto jointFact : jointsList[ w ] )
        {
          if( jointFact.fact.size() > 0 && jointFact.fact.find( bodyFact.name ) != std::string::npos )
          {
            jointsFacts.push_back( jointFact );
          }
        }

        //---build alternative facts
        std::string alternativeName = obstaclePrefix + std::to_string( obstacleId ) + "__from__" + bodyFact.name + "__world_" + std::to_string( w );

        // body alternative fact
        std::string bodyAlternativeFact = bodyFact.fact;
        boost::algorithm::replace_all( bodyAlternativeFact, bodyFact.name, alternativeName );

        // shape alternative fact
        std::string shapeAlternativeFact = shapeFact.fact;
        boost::algorithm::replace_all( shapeAlternativeFact, shapeFact.name, alternativeName );

        // joints alternative facts
        std::list < std::string > jointsAlternativeFacts;
        for( auto jointFact : jointsFacts )
        {
          std::string jointAlternativeFact = jointFact.fact;
          boost::algorithm::replace_all( jointAlternativeFact, jointFact.name, alternativeName );
          jointsAlternativeFacts.push_back( jointAlternativeFact );
        }

        // insert body laternative fact
        ss << bodyAlternativeFact;
        ss << shapeAlternativeFact;
        for( auto jointFact : jointsAlternativeFacts )
        {
          //std::cout << "jointFact:" << jointFact;
          ss << jointFact;
        }

        //
        obstacleId++;
      }
    }
  }

  // build the graph of the intersection
  //std::cout << "synthetized kin world:" << ss.str() << std::endl;

  Graph G;
  G.read( ss );

  mlr::KinematicWorld kin;
  kin.init( G );
  //kin.watch();

  return kin;
}

mlr::KinematicWorld * AONode::revertToRealKinematic( std::size_t w, mlr::KinematicWorld * optimized ) const
{
  mlr::KinematicWorld * reverted = new mlr::KinematicWorld();   // allocate here
  reverted->copy( *optimized );

  // filter bodies
//  for( auto b : optimized->bodies )
//  {
//    std::stringstream ss;
//    ss << b->name;
//    auto name = ss.str();

//    if( name.find( "__obstacle" ) != std::string::npos )
//    {
//      // delete body
//      // how??
//      //reverted->
//    }
//  }
//  std::stringstream ss;
//  optimized->write( ss );
//  Graph G;
//  G.read( ss );
//  optimized->init( G );

  /*auto factModifyer = [] ( const std::string& fact, std::list< std::string > & facts )
  {
    facts.push_back( fact );
    // unobservable fact
//    if( fact.find( "__obstacle" ) != std::string::npos ) // put as a keyword
//    {
//      //if(  )
//    }
//    else
//    {
//      facts.push_back( fact );
//    }
  };

  std::list< std::string > facts;
  for( auto b : optimized->bodies )
  {
    std::stringstream fact;
    fact <<"body " <<b->name <<" { ";
    b->write(fact);  fact <<" }\n";

    factModifyer( fact.str(), facts );
  }

  for( auto s : optimized->shapes )
  {
    std::stringstream fact;

    fact <<"shape ";
    if(s->name.N) fact <<s->name <<' ';
    fact <<"(" <<(s->body?(char*)s->body->name:"") <<"){ ";
    s->write(fact);  fact <<" }\n";

    factModifyer( fact.str(), facts );
  }

  for( auto j : optimized->joints )
  {
    std::stringstream fact;

    fact <<"joint ";
    if (j->name.N) fact <<j->name <<' ';
    fact <<"(" <<j->from->name <<' ' <<j->to->name <<"){ ";
    j->write(fact);  fact <<" }\n";

    factModifyer( fact.str(), facts );
  }

  // build world
  std::stringstream ss;
  for( auto fact : facts )
  {
    ss << fact << std::endl;
  }

  //std::cout << ss.str() << std::endl;

  Graph G;
  G.read( ss );
  reverted->init( G );
  reverted->checkConsistency();
  //reverted->watch();

  return reverted;
}*/

//===========================================================================

RUN_ON_INIT_BEGIN(manipulationTree)
AONodeL::memMove = true;
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
