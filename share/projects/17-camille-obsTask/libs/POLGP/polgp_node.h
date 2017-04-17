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
#include "geometric_level.h"

class POLGPNode;
struct ActionNode;
struct PlainMC;
struct MCStatistics;
typedef mlr::Array<ActionNode*> ActionNodeL;
typedef mlr::Array<POLGPNode*> POLGPNodeL;
typedef mlr::Array< mlr::Array<POLGPNode*> > POLGPNodeLL;

extern uint COUNT_kin, COUNT_evals, COUNT_poseOpt, COUNT_seqOpt, COUNT_pathOpt;

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
public:
  /// root node init
  POLGPNode( mlr::Array< std::shared_ptr< FOL_World > > fols, const mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > & kins, const arr & bs, const KOMOFactory & komoFactory );

  /// child node creation
  POLGPNode( POLGPNode *parent, double pHistory, const arr & bs, uint a );

  // modifiers
  void expand();
  void setAndSiblings( const mlr::Array< POLGPNode * > & siblings );
  void generateMCRollouts( uint num, int stepAbort );
  void backTrackBestExpectedPolicy();

  void solvePoseProblem();                        // strategy design pattern?
  void solveSeqProblem();
  void solvePathProblem( uint microSteps );
  void solveJointPathProblem( uint microSteps );

  //void labelInfeasible();

  // getters
  POLGPNode * parent() const { return parent_; }
  bool isExpanded() const { return isExpanded_; }
  POLGPNodeLL families() const { return families_; }
  bool isSymbolicallyTerminal() const { return isSymbolicallyTerminal_; }
  bool isSymbolicallySolved() const { return   isSymbolicallySolved_; }
  bool isPoseSolved() const { return poseProblem_.isSolved_; }
  bool isSequenceSolved() const { return seqProblem_.isSolved_; }
  bool isPathSolved() const { return pathProblem_.isSolved_; }
  bool isJointPathSolved() const { return jointProblem_.isSolved_; }

  int id() const { return id_; }
  POLGPNodeL bestFamily() const { return bestFamily_; }
  POLGPNodeL andSiblings() const { return andSiblings_; }
  double pHistory() const { return pHistory_; }
  bool isRoot() const { return parent_ == nullptr; }
  arr bs() const { return bs_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPoseProblems() const { return poseProblem_.komos_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoSeqProblems() const  { return seqProblem_.komos_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPathProblems() const { return pathProblem_.komos_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoJointPathProblems() const { return jointProblem_.komos_; }

  void labelInfeasible(); ///< sets the infeasible label AND removes all children!

  POLGPNodeL getTreePath();
  POLGPNodeL getTreePathFrom( POLGPNode * start );
  FOL_World::Handle & decision( uint w ) const { return decisions_( w ); }

  // utility
  std::string bestActionStr() const { return actionStr( expectedBestA_ ); }
  void indicateDifferentiatingFacts( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }
  std::set< std::string > differentiatingFacts() const { return differentiatingFacts_; }

private:
  void updateAndBacktrackPoseState();
  void updateAndBacktrackSequenceState();
  void updateAndBacktrackPathState();
  void updateAndBacktrackJointPathState();

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
  double time_;                                   ///< real time

  mlr::Array< POLGPNode * > andSiblings_;            /// at the same depth!
  mlr::Array< mlr::Array< POLGPNode * > > families_;
  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes

  mlr::Array< std::shared_ptr< PlainMC > > rootMCs_;
  MCStatistics * mcStats_;
  double expectedReward_;

  int expectedBestA_;
  mlr::Array< POLGPNode * > bestFamily_;

  //-- global search
  bool isExpanded_;
  bool isInfeasible_;

  //-- logic search
  bool isSymbolicallyTerminal_;           /// all the fol of this node are terminated
  bool isSymbolicallySolved_;             /// the children of this node are all solved

  //-- komo factory
  const KOMOFactory & komoFactory_;

  GeometricLevelType poseProblem_;
  GeometricLevelType seqProblem_;
  GeometricLevelType pathProblem_;
  GeometricLevelType jointProblem_;

  //--
  int id_;

  // parameters
  double maxConstraints_ = 0.5;
  double maxCost_        = 7.5;
};
