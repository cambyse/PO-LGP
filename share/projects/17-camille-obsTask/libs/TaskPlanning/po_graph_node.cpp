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


#include "po_graph_node.h"

#include <unordered_map>

#include <list>

#include <chrono>

#include <boost/algorithm/string/replace.hpp>


#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) x


//=====================free functions======================
static double eps() { return std::numeric_limits< double >::epsilon(); }
static double m_inf() { return -std::numeric_limits< double >::max(); }

namespace tp
{

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

static SymbolicState getStateStr( Graph * state )
{
  SymbolicState s;

  std::stringstream ss;
  state->write( ss," ","{}" );

  s.state = ss.str();

  for( auto node : * state )
  {
    //std::cout << * node << std::endl;

    std::stringstream ss;
    ss << * node;
    auto fact = ss.str();

    if( fact.find( "decision" ) == std::string::npos
        &&
        fact.find( "komo" ) == std::string::npos
        )
      s.facts.insert( fact );
  }

  return s;
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

static uint _n_get_actions;
static uint _get_actions_time_us;

static uint _n_transitions;
static uint _transition_time_us;

/// root node init
POGraphNode::POGraphNode( mlr::Array< std::shared_ptr< FOL_World > > fols, const arr & bs )
  : root_( nullptr )
  , N_( fols.N )
  , folWorlds_( fols )
  , folStates_( N_ )
  //, folAddToStates_( N_ )
  , pHistory_( 1.0 )
  , bs_( bs )
  //, a_( -1 )
  // global search
  , isExpanded_( false )
  , isInfeasible_( false )
  // logic search
  , isTerminal_( false )
  , isSolved_( false )
  //, lastActionReward_( 0 )
  //, prefixReward_( 0 )
  //, expectedTotalReward_( m_inf() )
  , expectedBestA_( -1 )
  , id_( 0 )
{
  for( auto w = 0; w < N_; ++w )
  {
    folWorlds_( w )->reset_state();
    folStates_( w ).reset( folWorlds_( w )->createStateCopy() );
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

std::list< POGraphNode::ptr > POGraphNode::graph_;

/// child node creation
POGraphNode::POGraphNode( const POGraphNode::ptr & root, double pHistory, const arr & bs,  const std::vector< SymbolicState > & resultStates, uint a )
  : root_( root )
  , N_( root->N_ )
  , folWorlds_( root->folWorlds_ )
  , folStates_( N_ )
  , resultStates_( resultStates )
  //, folAddToStates_( N_ )
  //, decisions_( N_ )
  , pHistory_( pHistory )
  , bs_( bs )
  //, a_( a )
  // global search
  , isExpanded_( false )
  , isInfeasible_( false )
  , isTerminal_( false )
  , isSolved_( false )
  // mc specific
  //, lastActionReward_( 0 )
  //, prefixReward_( 0 )
  //, expectedTotalReward_( m_inf() )
  , expectedBestA_ (-1 )
{
  // update the states
  bool isTerminal = true;
  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      mlr::String mlrState( resultStates_[ w ].state );
      // logic
      auto fol = folWorlds_( w );
      //fol->reset_state();
      fol->set_state( mlrState );

      folStates_( w ).reset( fol->createStateCopy() );

      bool isSubNodeTerminal = fol->successEnd;
      isTerminal = isTerminal && isSubNodeTerminal;

      if( fol->deadEnd )
      {
        isInfeasible_ = true;
      }
      //std::cout << *folStates_( w ) << std::endl;

      //folAddToStates_( w ) = nullptr;

      //decisions_( w ) = actions[ a_ ];
    }
  }
  isTerminal_ = isTerminal;

  if( isTerminal )
  {
    isSolved_ = true;
  }

  // update time
  //auto ls = getWitnessLogicAndState();
  //lastActionReward_ = ls.logic->lastStepReward;
  //prefixReward_ = parent_->prefixReward_ + lastActionReward_;

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

POGraphNode::L POGraphNode::expand()
{
  POGraphNode::L newNodes;

  //
  auto start = std::chrono::high_resolution_clock::now();
  //

  CHECK( ! isExpanded_, "" );
  if( isTerminal_ )
  {
    return newNodes;
  }

  // get possible actions for the worlds having a non null probability
  // retrieve actions for each world
  uint nActions = 0;
  std::vector< std::vector<FOL_World::Handle> > world_to_actions = getPossibleActions( nActions );

  if( nActions == 0 ) isTerminal_ = true;

  //std::cout << "number of possible actions:" << nActions << std::endl;

  for( auto a = 0; a < nActions; ++a )
  {
    //std::cout << "------------" << std::endl;
    //std::cout << "action:" << a << std::endl;
    std::unordered_map< std::set< std::string >, std::list< uint >, stringSetHash > outcomesToWorlds;
    std::vector< SymbolicState > resultStates( N_ );

    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > eps() )
      {
        auto logic = folWorlds_( w );
        auto state = folStates_( w );
        auto action = world_to_actions[ w ][ a ];
        //logic->addTerminalRule();
        //logic->reset_state();
        logic->setState( state.get() );
        logic->transition( action ); _n_transitions++;

        auto result             = logic->getState();

        auto stateStr           = getStateStr( result );
        auto observableStateStr = getObservableStateStr( result );

        //////////////////
        /*std::stringstream ss;
        ss << *action;
        auto action_label = ss.str();
        std::cout << "action_label:" << action_label << std::endl;
        if( action_label == "check" )
        {

        }

        if( logic->successEnd )
        {
          std::cout << "terminal logic found" << std::endl;
        }
        //std::cout << "result:" << *logic << std::endl;*/
        //////////////////

        resultStates[ w ] = stateStr;
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
    POGraphNode::L familiy;
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

      // find or create a node for each possible outcome
      POGraphNode::ptr child;
      bool found = false;

//      for( auto m = graph_.begin(); m != graph_.end(); ++m )
//      {
//        for( auto n = graph_.begin(); n != graph_.end(); ++n )
//        {
//          if( n != m )
//          {
//            bool eq = (*n)->bs() == (*m)->bs();
//            eq = eq && SymbolicState::equivalent( (*n)->resultStates(), (*m)->resultStates() );
//            CHECK( ! eq, "pb!!" );
//          }
//        }
//      }


      for( auto m = POGraphNode::graph_.begin(); m != POGraphNode::graph_.end(); ++m )
      {
        //std::cout << bs << "|" << (*m)->bs() << "|" << ((*m)->bs() == bs) << std::endl;

//        for( auto w = 0; w < bs.size(); ++w )
//        {
//          std::cout << (*m)->resultStates()[ w ].state << std::endl;
//          std::cout << resultStates[ w ].state         << std::endl;
//        }
        auto a = (*m)->resultStates();
        auto b = resultStates;

        if( (*m)->bs() == bs )
        if( SymbolicState::equivalent( a, b )  )
        {
          child = *m;
          found = true;
          break;
        }
      }

      if( ! found )
      {
        CHECK( child == nullptr, "a child was found but the pointer is still null!!" );

        child = std::make_shared< POGraphNode >( shared_from_this(), pWorld * pHistory_, bs, resultStates, a );
        POGraphNode::graph_.push_back( child );
        newNodes.push_back( child );

        // get the fact not in intersection
        std::set< std::string > differenciatingFacts;
        std::set_difference( facts.begin(), facts.end(), intersection.begin(), intersection.end(),
                             std::inserter(differenciatingFacts, differenciatingFacts.begin() ) );

        child->indicateDifferentiatingFacts( differenciatingFacts );
        //std::cout << "history:" << pHistory << " belief state:" << bs << " family size:" << familiy.d0 << std::endl;
      }
      familiy.append( child );
    }

    // check integrity
    double pSum = 0;
    for( auto n : familiy ) pSum += n->pHistory();

    CHECK_ZERO( pSum / pHistory() - 1.0, 0.000001, "" );
    //

    families_.append( familiy );

    // indicate and relation
    for( auto n : familiy )
    {
      n->setAndSiblings( familiy );
    }
  }

  isExpanded_ = true;

  //
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

  static int n; n++;
  if( n % 100 == 0 ) std::cout << "expansion time:" << ms << " |" << " get actions:" << _n_get_actions << " get actions time(ms):" << _get_actions_time_us / 1000 << " |" << " n transitions:" << _n_transitions << " transition time(ms):" << _transition_time_us / 1000 << " families:"<< families_.size() << std::endl;

  _n_transitions = 0;
  _transition_time_us = 0;

  _n_get_actions = 0;
  _get_actions_time_us = 0;

  return newNodes;
}

void POGraphNode::setAndSiblings( const POGraphNode::L & siblings )
{
  for( auto s : siblings )
  {
    if( s != shared_from_this() )
    andSiblings_.append( s );
  }
}

/*void POGraphNode::backTrackBestExpectedPolicy( POGraphNode::ptr until_node )
{
//  if( isTerminal() )
//  {
//    expectedReward_ = prefixReward_;  // it means that the future rewards is = 0 ( already terminal )
//    expectedBestA_  = -1;
//  }
//  else
//  {
    CHECK( ! isTerminal(), "nodes that are already terminal should not be listed as nodes to expand" );

    struct familyStatusType { double reward; bool solved; };
    mlr::Array< familyStatusType > familyStatus( families_.d0 );

    // find best family
    // compute cost of each family
    for( auto i = 0; i < families_.d0; ++i )
    {
      double familyReward = 0;
      bool   familySolved = true;

      for( auto c : families_( i ) )
      {
        //CHECK( c->lastActionReward_ == c->getWitnessLogicAndState().logic->lastStepReward, "" );
        familyReward += c->pHistory_ / pHistory_ * c->expectedTotalReward_;
        familySolved  = familySolved && c->isSolved_;
      }

      familyStatus( i ) = { familyReward, familySolved };
    }

    // sort
    double bestTotalReward = m_inf();
    int bestFamilyId = -1;
    for( auto i = 0; i < families_.d0; ++i )
    {
      if( familyStatus( i ).reward >= bestTotalReward )
      {
        bestTotalReward = familyStatus( i ).reward;
        bestFamilyId = i;
      }
    }

    // retrieve best decision id
    bestFamily_ = families_( bestFamilyId );
    uint bestA = bestFamily_.first()->a_;
    expectedBestA_        = bestA;
    isSolved_             = familyStatus( bestFamilyId ).solved;

    // check
    //std::cout << familyRewards << std::endl;
    //std::cout << actionStr( bestA ) << std::endl;
    //std::cout << "best family size:" << bestFamily_.d0 << " solved?" << familyStatus( bestFamilyId ).solved << std::endl;
    //
//  }

  if( parent_ && this != until_node.get() )
  {
    parent_->backTrackBestExpectedPolicy( until_node );
  }
}*/

/*void POGraphNode::backTrackSolveStatus()
{
  for( auto i = 0; i < families_.d0; ++i )
  {
    bool   familySolved = true;

    for( auto c : families_( i ) )
    {
      familySolved  = familySolved && c->isSolved_;
    }

    isSolved_ = isSolved_ || familySolved; // solved if at least of of the family os solved!
  }

  if( parent_ )
  {
    parent_->backTrackSolveStatus();
  }
}*/

void POGraphNode::labelInfeasible()
{
  // set flag and badest reward
  isInfeasible_ = true;
  //expectedTotalReward_ = m_inf();

  // delete children nodes
//  for( auto children : families_ )
//  {
//    DEL_INFEASIBLE( children.clear(); )
//  }
//  families_.clear();

  // backtrack results
  /*if( parent_ )
  {
    parent_->backTrackBestExpectedPolicy();
  }*/
}

/*POGraphNode::L POGraphNode::getTreePath()
{
  POGraphNode::L path;
  POGraphNode::ptr node = shared_from_this();
  for(;node;){
    path.append(node);
    node = node->parent_;
  }

  path.reverse();

  return path;
}

POGraphNode::L POGraphNode::getTreePathFrom( const POGraphNode::ptr & start )
{
  POGraphNode::L subPath;

  POGraphNode::ptr node = shared_from_this();
  do
  {
    subPath.prepend( node );
    node = node->parent_;

//    if( node == start )
//      std::cout << "node == start " << std::endl;

  } while ( ( node != start ) && node );

  return subPath;
}*/

//uint PONode::getPossibleActionsNumber() const
//{
//  auto logicAndState = getWitnessLogicAndState();

//  logicAndState.logic->setState( logicAndState.state.get(), d_ );

//  auto actions = logicAndState.logic->get_actions(); _n_get_actions++;

//  return actions.size();
//}

std::vector< std::vector<FOL_World::Handle> > POGraphNode::getPossibleActions( uint & nActions ) const
{
  std::vector< std::vector<FOL_World::Handle> > world_to_actions( N_ );

  //std::cout << "------------------------" << std::endl;

  for( auto w = 0; w < N_; ++w )
  {
    if( bs_( w ) > eps() )
    {
      auto logic = folWorlds_( w );
      auto state = folStates_( w );

      logic->setState( state.get() );

auto start_1 = std::chrono::high_resolution_clock::now();

      auto actions = folWorlds_( w )->get_actions(); _n_get_actions++;

auto elapsed_1 = std::chrono::high_resolution_clock::now() - start_1;
long long mcs_1 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_1).count();
_get_actions_time_us += mcs_1;

      world_to_actions[ w ] = actions;
      nActions = actions.size();

//      for( auto a : actions )
//      std::cout << *a << std::endl;
    }
  }

  return world_to_actions;
}

LogicAndState POGraphNode::getWitnessLogicAndState() const
{
  auto worlds = getPossibleLogicAndStates();

  CHECK( worlds.d0 > 0, "Unable to find a witness logic!!" );

  return worlds.first();
}

mlr::Array< LogicAndState > POGraphNode::getPossibleLogicAndStates() const
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

std::string POGraphNode::actionStr( uint a ) const
{
  auto ls = getWitnessLogicAndState();
  ls.logic->reset_state();
  ls.logic->setState( ls.state.get() );

  auto start_1 = std::chrono::high_resolution_clock::now();

      auto actions = ls.logic->get_actions(); _n_get_actions++;

  auto elapsed_1 = std::chrono::high_resolution_clock::now() - start_1;
  long long mcs_1 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_1).count();
  _get_actions_time_us += mcs_1;

  std::stringstream ss;

  if( a >= 0 && a < actions.size() )
    ss << *actions[a];
  else
    ss << "no actions";

  return ss.str();
}

//====free functions============//

namespace utility
{
POGraphNode::ptr getTerminalNode( const POGraphNode::ptr & n, const WorldID & w )
{
  POGraphNode::ptr node;
  if( n->isTerminal() )
  {
    CHECK( n->bs()( w.id() ) > eps(), "bug in getTerminalNode function, the belief state of the found node is invalid!" );
    node = n;
  }
  else
  {
    for( auto c : n->bestFamily() )
    {
      if( c->bs()( w.id() ) > eps() )
      {
        node = getTerminalNode( c, w );
        break;
      }
    }
  }

  return node;
}

void gatherPolicyFringe( const POGraphNode::ptr & node, std::set< mlr::Array< POGraphNode::ptr > > & fringe )
{
  for( auto f : node->families() )
  {
    if( f != node->bestFamily() )
    {
      fringe.insert( f );
    }
    else
    {
      for( auto c : f )
      {
        gatherPolicyFringe( c, fringe );
      }
    }
  }
}
}

//===========================================================================

}
