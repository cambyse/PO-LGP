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
  POGraphNode( const POGraphNode::ptr & root, double p, double pHistory, const arr & bs, const std::vector< SymbolicState > &,uint a );

  // modifiers
  POGraphNode::L expand();
  void setAndSiblings( const POGraphNode::L & siblings );
  void addParent( const POGraphNode::ptr & parent, uint actionId = -1 );
  POGraphNode::ptr root() { return root_ ? root_ : shared_from_this(); } // root_ is nullptr when the node itself is root

  // getters
  bool isExpanded() const { return isExpanded_; }
  POGraphNode::LL families() const { return families_; }
  bool isTerminal() const { return isTerminal_; }
  bool isSolved() const { return   isSolved_; }
  mlr::Array< std::shared_ptr<Graph> > folStates() const { return folStates_; }
  std::list< POGraphNode::ptr > graph() const { return graph_; }

  bool isRoot() const { return root_ == nullptr; }
  uint N() const { return N_; }
  uint id() const { return id_; }
  POGraphNode::L parents()     const { return parents_; }
  POGraphNode::L andSiblings() const { return andSiblings_; }
  double pHistory() const { return pHistory_; }
  double p() const { return p_; }
  arr bs()   const { return bs_; }
  std::vector< SymbolicState > resultStates() const { return resultStates_; }
  uint getLeadingAction( const POGraphNode::ptr & parent ) const;
  std::string getLeadingActionStr( const POGraphNode::ptr & parent ) const;
  // utility
  void indicateDifferentiatingFacts( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }
  std::set< std::string > differentiatingFacts() const { return differentiatingFacts_; }

private:
  // utility
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
  double p_;                              /// probability to jump to this node when there is an observation branching
  arr    bs_;

  POGraphNode::L  parents_;
  POGraphNode::L  andSiblings_;           /// at the same depth!
  POGraphNode::LL families_;
  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes

  mlr::Array< uint > leadingActions_;     /// potentially different because of different parents

  //-- global search
  bool isExpanded_;
  bool isInfeasible_;

  //-- logic search
  bool isTerminal_;           /// all the fol of this node are terminated
  bool isSolved_;             /// the children of this node are all solved

  //--
  uint id_;
};

namespace utility
{
  // free functions
  POGraphNode::ptr getTerminalNode( const POGraphNode::ptr &, const WorldID & w );
  void   gatherPolicyFringe( const POGraphNode::ptr &s, std::set< mlr::Array< POGraphNode::ptr> > & );
}

}
