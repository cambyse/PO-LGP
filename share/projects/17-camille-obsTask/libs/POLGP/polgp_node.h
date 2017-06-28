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

#include <Kin/kin.h>
#include <Logic/fol_mcts_world.h>
#include <LGP/LGP.h>
#include <Logic/fol.h>
#include <Motion/komo.h>
#include "komo_factory.h"
#include "geometric_level_base.h"
#include "node_visitor.h"

class POLGPNode;
struct ActionNode;
struct PlainMC;
struct MCStatistics;
typedef mlr::Array<ActionNode*> ActionNodeL;
typedef mlr::Array<POLGPNode*> POLGPNodeL;
typedef mlr::Array< mlr::Array<POLGPNode*> > POLGPNodeLL;

extern uint COUNT_kin, COUNT_evals, COUNT_poseOpt, COUNT_pathOpt;

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

struct LogicAndState
{
  std::shared_ptr< FOL_World > logic;
  std::shared_ptr< Graph >     state;
};

//===========================================================================

class POLGPNode
{
  friend class NodeVisitorBase;

public:
  /// root node init
  POLGPNode( mlr::Array< std::shared_ptr< FOL_World > > fols, const mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > & kins, const arr & bs, const KOMOFactory & komoFactory );

  /// child node creation
  POLGPNode( POLGPNode *parent, double pHistory, const arr & bs, uint a );

  // modifiers
  void expand();
  void setAndSiblings( const mlr::Array< POLGPNode * > & siblings );
  void generateMCRollouts( uint num, int stepAbort );
  void backTrackBestExpectedPolicy( POLGPNode * node = nullptr ); // backtrack up to the node node, per default, backup up to root

  void registerGeometricLevel( GeometricLevelBase::ptr const& );

  void solvePoseProblem();                        // strategy design pattern?
  void solvePathProblem();
  void solveJointPathProblem();
  void labelInfeasible(); ///< sets the infeasible label AND removes all children!
  //void resetSymbolicallySolved() { isSymbolicallySolved_ = false; }

  void acceptVisitor( NodeVisitorBase & visitor ) { visitor.visit( this ); } // overkill here, visitor design pattern usefull if we have a hierarchy of class!
  //void labelInfeasible();

  // getters
  POLGPNode * parent() const { return parent_; }
  bool isExpanded() const { return isExpanded_; }
  POLGPNodeLL families() const { return families_; }
  bool isSymbolicallyTerminal() const { return isSymbolicallyTerminal_; }
  bool isSymbolicallySolved() const { return   isSymbolicallySolved_; }
  bool isPoseSolved() const { return poseProblem_->isSolved(); }
  bool isPathSolved() const { return pathProblem_->isSolved(); }
  bool isJointPathSolved() const { return jointProblem_->isSolved(); }

  uint N() const { return N_; }
  int id() const { return id_; }
  POLGPNodeL bestFamily() const { return bestFamily_; }
  POLGPNodeL andSiblings() const { return andSiblings_; }
  double pHistory() const { return pHistory_; }
  bool isRoot() const { return parent_ == nullptr; }
  arr bs() const { return bs_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPoseProblems() const { return poseProblem_->komos(); }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPathProblems() const { return pathProblem_->komos(); }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoJointPathProblems() const { return jointProblem_->komos(); }

  POLGPNodeL getTreePath();
  POLGPNodeL getTreePathFrom( POLGPNode * start );
  FOL_World::Handle & decision( uint w ) const { return decisions_( w ); }

  // for geometric levels
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > startKinematics() const { return startKinematics_; }
  mlr::Array< mlr::KinematicWorld > & effKinematics() { return effKinematics_; }
  mlr::Array< std::shared_ptr<Graph> > folStates() const { return folStates_; }
  GeometricLevelBase::ptr poseGeometricLevel() const { return poseProblem_; }
  GeometricLevelBase::ptr pathGeometricLevel() const { return pathProblem_; }
  GeometricLevelBase::ptr jointPathGeometricLevel() const { return jointProblem_; }

  double time() const { return time_; }
  double prefixReward() const { return prefixReward_; }
  double expecteTotalReward() const { return expectedReward_ ; }
  double expecteFutureReward() const { return expectedReward_ - prefixReward_; }

  // utility
  std::string bestActionStr() const { return actionStr( expectedBestA_ ); }
  std::string leadingActionStr() const { return parent_->actionStr( a_ ); }
  void indicateDifferentiatingFacts( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }
  std::set< std::string > differentiatingFacts() const { return differentiatingFacts_; }

private:
  // utility
  uint getPossibleActionsNumber() const;
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
  POLGPNode * parent_;

  // members for symbolic search
  uint N_;                                                                    ///< number of possible worlds
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  mlr::Array< std::shared_ptr<Graph> >     folStates_;                        ///< INITIAL fol state, state when the PARENT action has been executed
  //mlr::Array< Graph* >  folAddToStates_; ///< facts that are added to the state /after/ the fol.transition, e.g., infeasibility predicates

  //-- kinematics: the kinematic structure of the world after the decision path
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > startKinematics_; ///< initial start state kinematics
  mlr::Array< mlr::KinematicWorld > effKinematics_;                            ///< the effective kinematics (computed from kinematics and symbolic state)

  double pHistory_;
  arr bs_;

  int a_;                                         ///< action id that leads to this node
  mlr::Array< FOL_World::Handle > decisions_;     ///< actions leading to this node ( one for each logic )

  uint d_;                                        ///< decision depth/step of this node
  double time_;                                   ///< real time, root = 0, represents the end of the parent action

  mlr::Array< POLGPNode * > andSiblings_;            /// at the same depth!
  mlr::Array< mlr::Array< POLGPNode * > > families_;
  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes

  mlr::Array< std::shared_ptr< PlainMC > > rootMCs_;
  MCStatistics * mcStats_;
  double lastActionReward_;                       ///  reward of the action leading to this node
  double prefixReward_;                           ///  this is the (certain) rewards of the prefix decisions
  double expectedReward_;                         ///  the total expected reward ?

  int expectedBestA_;                             ///  expected next best action
  mlr::Array< POLGPNode * > bestFamily_;

  //-- global search
  bool isExpanded_;
  bool isInfeasible_;

  //-- logic search
  bool isSymbolicallyTerminal_;           /// all the fol of this node are terminated
  bool isSymbolicallySolved_;             /// the children of this node are all solved

  GeometricLevelBase::ptr poseProblem_;
  GeometricLevelBase::ptr pathProblem_;
  GeometricLevelBase::ptr jointProblem_;

  std::map< mlr::String, GeometricLevelBase::ptr > geometricLevels_;

  //--
  int id_;
};

namespace utility
{
  // free functions
  POLGPNode * getTerminalNode( POLGPNode *, const WorldID & w );
}
