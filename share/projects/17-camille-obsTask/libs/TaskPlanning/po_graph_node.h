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

#pragma once

#include <set>
#include <list>

#include <Kin/kin.h>
#include <Logic/fol_mcts_world.h>
#include <Logic/fol.h>
#include "node_visitor.h"
#include "utility.h"

namespace tp
{

//===========================================================================

struct SymbolicState
{
  static bool equivalent( const std::vector< SymbolicState > & a, const std::vector< SymbolicState > & b )
  {
    bool ret = true;

    CHECK( a.size() == b.size(), "checking the equivalency of two symbolic states that are not comparable!!" )

    for( auto w = 0; w < a.size(); ++w )
    {
      ret = ret && equivalent( a[ w ], b[ w ] );
    }

    return ret;
  }

  static bool equivalent( const SymbolicState & a, const SymbolicState & b )
  {
    a.facts == b.facts;
  }

  std::string state;
  std::set< std::string > facts;
};

//===========================================================================

class WorldID
{
public:
  explicit WorldID( std::size_t id )
    : id_( id )
  {

  }

  std::size_t id() const { return id_; }

private:
  std::size_t id_;
};

//===========================================================================

class POGraphNode : public std::enable_shared_from_this< POGraphNode > // note: public inheritance
{
  friend class NodeVisitorBase;
public:
  typedef std::shared_ptr< POGraphNode > ptr;
  typedef mlr::Array< POGraphNode::ptr > L;
  typedef mlr::Array< mlr::Array<POGraphNode::ptr > > LL;

public:
  /// root node init
  POGraphNode( mlr::Array< std::shared_ptr< FOL_World > > fols, const arr & bs );

  /// child node creation
  POGraphNode( const POGraphNode::ptr & root, double pHistory, const arr & bs, const std::vector< SymbolicState > &,uint a );

  // modifiers
  POGraphNode::L expand();
  void setAndSiblings( const POGraphNode::L & siblings );
  void setBestFamily ( const POGraphNode::L & f ) { bestFamily_ = f; /*expectedBestA_ = f( 0 )->a_;*/ }
  //void backTrackBestExpectedPolicy( POGraphNode::ptr until_node = nullptr ); // backtrack up to the node node, per default, backup up to root

  //void backTrackSolveStatus(); // backtrack up to the node node, per default, backup up to root

  void labelInfeasible(); ///< sets the infeasible label, should remove all children?
  //void resetSymbolicallySolved() { isSymbolicallySolved_ = false; }

  //void acceptVisitor( NodeVisitorBase & visitor ) { visitor.visit( shared_from_this() ); } // overkill here, visitor design pattern usefull if we have a hierarchy of class!
  //void labelInfeasible();

  // getters
  //POGraphNode::ptr parent() const { return parent_; }
  bool isExpanded() const { return isExpanded_; }
  POGraphNode::LL families() const { return families_; }
  bool isTerminal() const { return isTerminal_; }
  bool isSolved() const { return   isSolved_; }
  mlr::Array< std::shared_ptr<Graph> > folStates() const { return folStates_; }
  std::list< POGraphNode::ptr > graph() const { return graph_; }

  uint N() const { return N_; }
  int id() const { return id_; }
  POGraphNode::L bestFamily() const { return bestFamily_; }
  POGraphNode::L andSiblings() const { return andSiblings_; }
  double pHistory() const { return pHistory_; }
  bool isRoot() const { return this == root_.get(); }
  arr bs() const { return bs_; }
  std::vector< SymbolicState > resultStates() const { return resultStates_; }

  /*POGraphNode::L getTreePath();
  POGraphNode::L getTreePathFrom( const POGraphNode::ptr & start );*/
  //FOL_World::Handle & decision( uint w ) const { return decisions_( w ); }

  //double prefixReward() const { return prefixReward_; }
  //double expecteTotalReward() const { return expectedTotalReward_; }
  //double expecteFutureReward() const { return expectedTotalReward_ - prefixReward_; }

  // utility
  std::string bestActionStr() const { return actionStr( expectedBestA_ ); }
  //std::string leadingActionStr() const { return parent_->actionStr( a_ ); }
  void indicateDifferentiatingFacts( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }
  std::set< std::string > differentiatingFacts() const { return differentiatingFacts_; }

private:
  // utility
  uint getPossibleActionsNumber() const;
  std::vector< std::vector<FOL_World::Handle> > getPossibleActions( uint & nActions ) const;
  LogicAndState getWitnessLogicAndState() const;
  template < typename T > T getWitnessElem( const mlr::Array< T > array ) const
  {
    CHECK( array.d0 == N_, "wrong dimensions!" );
    for( auto w = 0; w < N_; ++w )
    {
      if( bs_( w ) > std::numeric_limits< double >::epsilon() )
      {
        return array( w );
      }
    }
  }

  mlr::Array< LogicAndState > getPossibleLogicAndStates() const;
  std::string actionStr( uint ) const;

private:
  POGraphNode::ptr root_;

  // members for symbolic search
  uint N_;                                                                    ///< number of possible worlds
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  mlr::Array< std::shared_ptr<Graph> >     folStates_;                        ///< INITIAL fol state, state when the PARENT action has been executed
  std::vector< SymbolicState >             resultStates_;
  static std::list< POGraphNode::ptr >     graph_;


  double pHistory_;
  arr    bs_;

  //int a_;                                         ///< action id that leads to this node
  //mlr::Array< FOL_World::Handle > decisions_;     ///< actions leading to this node ( one for each logic )

  POGraphNode::L  andSiblings_;            /// at the same depth!
  POGraphNode::LL families_;
  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes

  //double lastActionReward_;                       ///  reward of the action leading to this node
  //double prefixReward_;                           ///  this is the (certain) rewards of the prefix decisions
  //double expectedTotalReward_;                   ///  the expected future reward ?
  //double expectedFutureReward_;                   ///  the expected future reward ?

  int expectedBestA_;                             ///  expected next best action
  POGraphNode::L bestFamily_;

  //-- global search
  bool isExpanded_;
  bool isInfeasible_;

  //-- logic search
  bool isTerminal_;           /// all the fol of this node are terminated
  bool isSolved_;             /// the children of this node are all solved

  //--
  int id_;
};

namespace utility
{
  // free functions
  POGraphNode::ptr getTerminalNode( const POGraphNode::ptr &, const WorldID & w );
  void   gatherPolicyFringe( const POGraphNode::ptr &s, std::set< mlr::Array< POGraphNode::ptr> > & );
}

}
