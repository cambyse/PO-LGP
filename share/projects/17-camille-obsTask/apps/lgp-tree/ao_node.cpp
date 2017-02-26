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

// outcome structure
//struct outcomeType
//{
//  //int folWorldID;
//  std::string observableState;
//};

//bool operator==( const outcomeType & a, const outcomeType & b )
//{
//  return ( /*a.folWorldID == b.folWorldID &&*/
//           a.actionID   == b.actionID &&
//           a.observableState == b.observableState
//           );
//};

//struct outcomeHash {
//size_t operator()( const outcomeType & outcome ) const
//{
//    return std::hash<int>()( outcome.actionID ) ^ std::hash<std::string>()( outcome.observableState );
//}
//};

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
AONode::AONode( mlr::Array< std::shared_ptr< FOL_World > > fols, const arr & bs, const KOMOFactory & komoFactory )
  : parent_( nullptr )
  , folWorlds_( fols )
  , folStates_( folWorlds_.d0 )
  , pHistory_( 1.0 )
  , bs_( bs )
  , a_( -1 )
  , d_( 0 )
  , isExpanded_( false )
  , isTerminal_( false )
  , isSolved_( false )
  , isInfeasible_( false )
  , rootMCs_( folWorlds_.d0 )
  , mcStats_( new MCStatistics )
  , expectedReward_( 0 )
  , expectedBestA_( -1 )
  , komoFactory_( komoFactory )
  , id_( 0 )
{
  for( auto w = 0; w < folWorlds_.d0; ++w )
  {
    folWorlds_( w )->reset_state();
    folStates_( w ).reset( folWorlds_( w )->createStateCopy() );
    rootMCs_( w ).reset( new PlainMC( *folWorlds_( w ) ) );
    rootMCs_( w )->verbose = 0;
  }
}

/// child node creation
AONode::AONode(AONode *parent, double pHistory, const arr & bs, uint a )
  : parent_( parent )
  , folWorlds_( parent->folWorlds_ )
  , folStates_( folWorlds_.d0 )
  , decisions_( folWorlds_.d0 )
  , pHistory_( pHistory )
  , bs_( bs )
  , a_( a )
  , d_( parent->d_ + 1 )
  , isExpanded_( false )
  , isTerminal_( false )
  , isSolved_( false )
  , isInfeasible_( false )
  , rootMCs_( parent->rootMCs_ )
  , mcStats_( new MCStatistics )
  , expectedReward_( 0 )
  , expectedBestA_( -1 )
  , komoFactory_( parent->komoFactory_ )
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
    isSolved_ = true;

  if( isTerminal_ )
  {
    std::cout << "found terminal node!" << bs_ << std::endl;
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
      std::set_intersection( intersection.begin(), intersection.end(),
                             facts.begin(), facts.end(),
                             std::inserter( intersection, intersection.begin() ) );
    }

    // create as many children as outcomes
    mlr::Array< AONode * > familiy;
    for( auto outcome : outcomesToWorlds )
    {
      auto facts  = outcome.first;
      auto worlds = outcome.second;

      // update belief state
      arr bs = zeros( bs_.d0 );
      double pHistory = 0;
      for( auto w : worlds )
      {
        pHistory += bs_( w );
        bs( w ) = bs_( w );
      }

      bs = bs / pHistory;

      CHECK( pHistory > 0, "wrong node expansion" );

      // create a node for each possible outcome

      auto n = new AONode( this, pHistory, bs, a );
      familiy.append( n );

      // get the fact not in intersection
      std::set< std::string > differenciatingFacts;
      std::set_difference( facts.begin(), facts.end(), intersection.begin(), intersection.end(),
                           std::inserter(differenciatingFacts, differenciatingFacts.begin() ) );

      n->indicateDifferentiatingFacts( differenciatingFacts );
      //std::cout << "history:" << pHistory << " belief state:" << bs << " family size:" << familiy.d0 << std::endl;
    }

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
        familyReward += c->pHistory_ * c->expectedReward_;
        familySolved = familySolved && c->isSolved_;
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
    isSolved_ = familyStatus( bestFamilyId ).solved;

    // check
    //std::cout << familyRewards << std::endl;
    //std::cout << actionStr( bestA ) << std::endl;
    //std::cout << "best family size:" << bestFamily_.d0 << " solved?" << familyStatus( bestFamilyId ).solved << std::endl;
    //
  }

  if( parent_ )
    parent_->backTrackBestExpectedPolicy();
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

//===========================================================================

RUN_ON_INIT_BEGIN(manipulationTree)
AONodeL::memMove = true;
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
