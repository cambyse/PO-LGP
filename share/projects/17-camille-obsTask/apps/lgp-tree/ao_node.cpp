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


//===========================================================================

static int nodeNumber = 0;

/// root node init
AONode::AONode( mlr::Array< std::shared_ptr< FOL_World > > fols, const mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > & kins, const arr & bs, const KOMOFactory & komoFactory )
  : parent_( nullptr )
  , folWorlds_( fols )
  , folStates_( folWorlds_.d0 )
  , startKinematics_( kins )
  , effKinematics_( kins.d0 )
  , pHistory_( 1.0 )
  , bs_( bs )
  , a_( -1 )
  , d_( 0 )
  , time_( 0.0 )
  , isExpanded_( false )
  , isTerminal_( false )
  , isSymbolicallySolved_( false )
  , isInfeasible_( false )
  , rootMCs_( folWorlds_.d0 )
  , mcStats_( new MCStatistics )
  , expectedReward_( 0 )
  , expectedBestA_( -1 )
  , komoFactory_( komoFactory )
  , poseCost_( 0.0 )
  , poseConstraints_( 0.0 )
  , poseFeasible_( false )
  , komoPoseProblems_( kins.d0 )
  , komoSeqProblems_( kins.d0 )
  , komoPathProblems_( kins.d0 )
  , id_( 0 )
{
  for( auto w = 0; w < folWorlds_.d0; ++w )
  {
    folWorlds_( w )->reset_state();
    folStates_( w ).reset( folWorlds_( w )->createStateCopy() );
    rootMCs_( w ).reset( new PlainMC( *folWorlds_( w ) ) );
    rootMCs_( w )->verbose = 0;
  }

  for( auto w = 0; w < startKinematics_.d0; ++w )
  {
    effKinematics_( w ) = mlr::KinematicWorld( * startKinematics_( w ) );
  }
}

/// child node creation
AONode::AONode(AONode *parent, double pHistory, const arr & bs, uint a )
  : parent_( parent )
  , folWorlds_( parent->folWorlds_ )
  , folStates_( folWorlds_.d0 )
  , startKinematics_( parent->startKinematics_ )
  , effKinematics_( parent->effKinematics_ )
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
  , poseCost_( 0.0 )
  , poseConstraints_( 0.0 )
  , poseFeasible_( false )
  , komoPoseProblems_( parent->komoPoseProblems_.d0 )
  , komoSeqProblems_( parent->komoSeqProblems_.d0 )
  , komoPathProblems_( parent->komoPathProblems_.d0 )
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

  if( isTerminal_ )
  {
    std::cout << "found terminal node!" << bs_ << std::endl;
  }

  // update time
  auto ls = getWitnessLogicAndState();
  time_ = parent_->time_ + ls.logic->lastStepDuration;

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
  bool sameTrajectories = sameAgentTrajectories( komoPoseProblems_ );

  if( ! sameTrajectories )
    labelInfeasible();

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
  bool sameTrajectories = sameAgentTrajectories( komoSeqProblems_ );

  if( ! sameTrajectories )
    labelInfeasible();

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

      for( auto node:treepath ){
        komo->groundTasks((node->parent_?node->parent_->time_:0.), *node->folStates_( w ) );
      }

      DEBUG( FILE("z.fol") <<fol; )
      DEBUG( komo->MP->reportFeatures(true, FILE("z.problem")); )
      komo->reset();
      try{
        komo->run();
      } catch(const char* msg){
        cout << "KOMO FAILED: " << msg <<endl;
      }
    }
  }

  // check if all worlds lead to same agent sequence of positions
  bool sameTrajectories = sameAgentTrajectories( komoPathProblems_ );

  if( ! sameTrajectories )
    labelInfeasible();

  // all the komo lead to the same agent trajectory, its ok to use one of it for the rest
  auto komo = getWitnessPathKomo();
  //komo->displayTrajectory();
  COUNT_evals += komo->opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_pathOpt++;

  DEBUG( komoPathProblem->MP->reportFeatures(true, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komo->getReport();
  DEBUG( FILE("z.problem.cost") << result; )
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if( ! path_.N || cost < pathCost_ )
  {
    pathCost_ = cost;
    pathConstraints_ = constraints;
    pathFeasible_ = (constraints<.5);
    path_ = komo->x;
  }

  if( ! pathFeasible_ )
    labelInfeasible();
}

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

bool AONode::sameAgentTrajectories( const mlr::Array< ExtensibleKOMO::ptr > & komos )
{
  bool sameAgentTrajectories = true;
  mlr::Array< arr >  agentConfigurations;
  const double gentleEps = 0.001;
  for( auto w = 1; w < komos.d0; ++w )
  {
    if( bs_( w ) > eps() )
    {
      auto configurations = komos( w )->MP->configurations;
      auto qA = startKinematics_( w )->q_agent;
      auto nA = startKinematics_( w )->getJointStateDimension( qA );

      // copy first config
      if( agentConfigurations.d0 == 0 )
      {
        agentConfigurations = mlr::Array< arr >( configurations.d0 );

        for( auto s = 0; s < configurations.d0; ++s )
        {
          agentConfigurations( s ) = arr( nA );

          for( auto i = 0; i < nA; ++i )
          {
            agentConfigurations( s )( i ) = configurations( s )->q( qA + i );
          }
        }
      }
      // if we allready have a ref, we compare
      else
      {
        for( auto s = 0; s < agentConfigurations.d0; ++s )
        {
          for( auto i = 0; i < nA; ++i )
          {
            double refVal     = agentConfigurations( s )( i );
            double checkedVal = configurations( s )->q( qA + i );
            //std::cout << "refVal:" << refVal << " checkVal:" << checkedVal << std::endl;
            sameAgentTrajectories = sameAgentTrajectories && ( fabs( refVal - checkedVal ) < gentleEps );
            //if( !sameAgentConfigurations )
            //  std::cout << "different conf!!" << std::endl;
          }
        }
      }
    }
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

//mlr::KinematicWorld AONode::getStartKinematic() const
//{
//  // temporary : here we should build a kin world containing only the intersection of the facts??
//  // tmp camille
//  mlr::KinematicWorld world;

//  // if the node is root
//  if( isRoot() )
//  {
//    world = *startKinematics_.first();
//  }
//  // else
//  else
//  {
//    for( auto w = 0; w < effKinematics_.d0; ++w )
//    {
//      if( bs_( w ) > eps() )
//      {
//        world = parent_->effKinematics_( w );
//        break;
//      }
//    }
//  }

//  return world;
//}

//===========================================================================

RUN_ON_INIT_BEGIN(manipulationTree)
AONodeL::memMove = true;
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
