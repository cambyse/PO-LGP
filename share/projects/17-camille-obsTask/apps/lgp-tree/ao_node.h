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
#include "action_node.h"

class AONode;
struct ActionNode;
struct PlainMC;
struct MCStatistics;
typedef mlr::Array<ActionNode*> ActionNodeL;
typedef mlr::Array<AONode*> AONodeL;
typedef mlr::Array< mlr::Array<AONode*> > AONodeLL;

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

class AONode
{
public:
  /// root node init
  AONode( mlr::Array< std::shared_ptr< FOL_World > > fols, const mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > & kins, const arr & bs, const KOMOFactory & komoFactory );

  /// child node creation
  AONode( AONode *parent, double pHistory, const arr & bs, uint a );

  // modifiers
  void expand();
  void setAndSiblings( const mlr::Array< AONode * > & siblings );
  void generateMCRollouts( uint num, int stepAbort );
  void backTrackBestExpectedPolicy();

  void solvePoseProblem();
  void solveSeqProblem();
  void solvePathProblem( uint microSteps );
  void solveJointPathProblem( uint microSteps );

  void labelInfeasible();

  // getters
  AONode * parent() const { return parent_; }
  bool isExpanded() const { return isExpanded_; }
  AONodeLL families() const { return families_; }
  bool isSymbolicallyTerminal() const { return isSymbolicallyTerminal_; }
  bool isSymbolicallySolved() const { return   isSymbolicallySolved_; }
  bool isPoseSolved() const { return isPoseProblemSolved_; }
  bool isSequenceSolved() const { return isSequenceProblemSolved_; }
  bool isPathSolved() const { return isPathProblemSolved_; }
  bool isJointPathSolved() const { return isJointPathProblemSolved_; }

  int id() const { return id_; }
  AONodeL bestFamily() const { return bestFamily_; }
  AONodeL andSiblings() const { return andSiblings_; }
  double pHistory() const { return pHistory_; }
  bool isRoot() const { return parent_ == nullptr; }
  arr bs() const { return bs_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPoseProblems() const { return komoPoseProblems_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoSeqProblems() const  { return komoSeqProblems_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPathProblems() const { return komoPathProblems_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoJointPathProblems() const { return komoJointPathProblems_; }

  void labelInfeasible( uint w ); ///< sets the infeasible label AND removes all children!

  AONodeL getTreePath();
  AONodeL getTreePathFrom( AONode * start );
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
  AONode * parent_;

  // members for symbolic search
  uint N_;                                                                    ///< number of possible worlds
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  mlr::Array< std::shared_ptr<Graph> >     folStates_;
  mlr::Array< Graph* >  folAddToStates_; ///< facts that are added to the state /after/ the fol.transition, e.g., infeasibility predicates

  //-- kinematics: the kinematic structure of the world after the decision path
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > startKinematics_; ///< initial start state kinematics
  mlr::Array< mlr::KinematicWorld > effKinematics_;                            ///< the effective kinematics (computed from kinematics and symbolic state)

  double pHistory_;
  arr bs_;

  int a_;                                         ///< action id that leads to this node
  mlr::Array< FOL_World::Handle > decisions_;     ///< actions leading to this node ( one for each logic )

  uint d_;                                        ///< decision depth/step of this node
  double time_;                                   ///< real time

  mlr::Array< AONode * > andSiblings_;            /// at the same depth!
  mlr::Array< mlr::Array< AONode * > > families_;
  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes

  mlr::Array< std::shared_ptr< PlainMC > > rootMCs_;
  MCStatistics * mcStats_;
  double expectedReward_;

  int expectedBestA_;
  mlr::Array< AONode * > bestFamily_;

  //-- global search
  bool isExpanded_;
  bool isInfeasible_;
  bool isTerminal_;
  bool isSolved_;

  //-- logic search
  bool isSymbolicallyTerminal_;
  bool isSymbolicallySolved_;

  //-- komo factory
  const KOMOFactory & komoFactory_;

  //-- pose opt
  mlr::Array< double > poseCosts_;        ///< costs of the poses from root node up to this node
  mlr::Array< double > poseConstraints_;  ///< costs of the constarints from root node up to this node
  mlr::Array< bool >   poseSolved_;
  mlr::Array< bool >   poseFeasibles_;    ///< indicates wether the optimization is feasible on this node only
  mlr::Array< ExtensibleKOMO::ptr > komoPoseProblems_; ///< komo object used to optimize poses
  bool isPoseTerminal_;
  bool isPoseProblemSolved_;              ///< is set to true if all all the paths passing through this node are solved

  //-- sequence opt
  mlr::Array< double > seqCosts_;
  mlr::Array< double > seqConstraints_;
  mlr::Array< bool >   seqSolved_;
  mlr::Array< bool >   seqFeasibles_;
  mlr::Array< ExtensibleKOMO::ptr > komoSeqProblems_;
  bool isSequenceTerminal_;
  bool isSequenceProblemSolved_;

  //-- path opt
  mlr::Array< double > pathCosts_;
  mlr::Array< double > pathConstraints_;
  mlr::Array< bool >   pathFeasibles_;
  mlr::Array< ExtensibleKOMO::ptr > komoPathProblems_;
  mlr::Array< WorldL > pathConfigurations_;
  bool isPathTerminal_;
  bool isPathProblemSolved_;

  //-- joint path opt
  mlr::Array< double > jointPathCosts_;
  mlr::Array< double > jointPathConstraints_;
  mlr::Array< bool >   jointPathFeasibles_;
  mlr::Array< ExtensibleKOMO::ptr >  komoJointPathProblems_;
  mlr::Array< WorldL > jointPathConfigurations_;
  bool isJointPathTerminal_;
  bool isJointPathProblemSolved_;

  //--
  int id_;
};
