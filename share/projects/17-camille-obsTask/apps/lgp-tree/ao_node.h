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

struct LogicAndState
{
  std::shared_ptr< FOL_World > logic;
  std::shared_ptr< Graph >     state;
};

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
  void solvePathProblem2( uint microSteps, AONode * start );

  void labelInfeasible();

  // getters
  AONode * parent() const { return parent_; }
  bool isExpanded() const { return isExpanded_; }
  AONodeLL families() const { return families_; }
  bool isTerminal() const { return isTerminal_; }
  bool isSolved() const { return isSymbolicallySolved_; }
  int id() const { return id_; }
  AONodeL bestFamily() const { return bestFamily_; }
  AONodeL andSiblings() const { return andSiblings_; }
  double pHistory() const { return pHistory_; }
  bool isRoot() const { return parent_ == nullptr; }
  arr bs() const { return bs_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPoseProblems() const { return komoPoseProblems_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoSeqProblems() const  { return komoSeqProblems_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPathProblems() const { return komoPathProblems_; }
  mlr::Array< std::shared_ptr<ExtensibleKOMO> > komoPathProblems2() const { return komoPathProblems2_; }
  mlr::Array< WorldL > path2Configurations() const { return path2Configurations_; }

  AONodeL getTreePath();
  AONodeL getTreePathFrom( AONode * start );
  FOL_World::Handle & decision( uint w ) const { return decisions_( w ); }

  // utility
  std::string bestActionStr() const { return actionStr( expectedBestA_ ); }
  void indicateDifferentiatingFacts( const std::set< std::string > & facts ) { differentiatingFacts_ = facts; }
  std::set< std::string > differentiatingFacts() const { return differentiatingFacts_; }

private:
  bool sameAgentTrajectories( const mlr::Array< ExtensibleKOMO::ptr > & komos );
  uint getPossibleActionsNumber() const;
  LogicAndState getWitnessLogicAndState() const;
  template < typename T > T getWitnessElem( const mlr::Array< T > array ) const
  {
    CHECK( array.d0 == bs_.d0, "wrong dimensions!" );
    for( auto w = 0; w < array.d0; ++w )
    {
      if( bs_( w ) > std::numeric_limits< double >::epsilon() )
      {
        return array( w );
      }
    }
  }
  ExtensibleKOMO::ptr getWitnessPoseKomo()     const { return getWitnessElem( komoPoseProblems_ ); }
  ExtensibleKOMO::ptr getWitnessSeqKomo()      const { return getWitnessElem( komoSeqProblems_ );  }
  ExtensibleKOMO::ptr getWitnessPathKomo()     const { return getWitnessElem( komoPathProblems_ ); }
  ExtensibleKOMO::ptr getWitnessPathKomo2()    const { return getWitnessElem( komoPathProblems2_ ); }

  mlr::Array< LogicAndState > getPossibleLogicAndStates() const;
  std::string actionStr( uint ) const;

  //mlr::Array< mlr::KinematicWorld > getPossibleKinematicWorlds() const;
//  mlr::KinematicWorld getStartKinematic() const;

private:
  AONode * parent_;

  // members for symbolic search
  mlr::Array< std::shared_ptr<FOL_World> > folWorlds_;
  mlr::Array< std::shared_ptr<Graph> >     folStates_;

  //-- kinematics: the kinematic structure of the world after the decision path
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > startKinematics_; ///< initial start state kinematics
  mlr::Array< mlr::KinematicWorld > effKinematics_;         ///< the effective kinematics (computed from kinematics and symbolic state)
  mlr::Array< mlr::KinematicWorld > effKinematicsPaths2_;         ///< the effective kinematics (computed from kinematics and symbolic state)
  mlr::Array< bool > effKinematicsPaths2areSet_;

  double pHistory_;
  arr bs_;

  int a_;                                    ///< action id that leads to this node
  mlr::Array< FOL_World::Handle > decisions_;///< actions leading to this node ( one for each logic )

  uint d_;                                   ///< decision depth/step of this node
  double time_;                              ///< real time

  mlr::Array< AONode * > andSiblings_;  /// on the same depth!
  mlr::Array< mlr::Array< AONode * > > families_;
  std::set< std::string > differentiatingFacts_;  ///< used only for debugging purposes

  mlr::Array< std::shared_ptr< PlainMC > > rootMCs_;
  MCStatistics * mcStats_;
  double expectedReward_;

  int expectedBestA_;
  mlr::Array< AONode * > bestFamily_;

  //-- status flags
  bool isExpanded_;
  bool isTerminal_;
  bool isSymbolicallySolved_;
  bool isInfeasible_;

  //-- komo factory
  const KOMOFactory & komoFactory_;

  //-- pose opt
  double poseCost_, poseConstraints_;
  bool poseFeasible_;
  arr pose_;
  mlr::Array< ExtensibleKOMO::ptr > komoPoseProblems_;

  //-- sequence opt
  double seqCost_, seqConstraints_;
  bool seqFeasible_;
  arr seq_;
  mlr::Array< ExtensibleKOMO::ptr > komoSeqProblems_;

  //-- path opt
  double pathCost_, pathConstraints_;
  bool pathFeasible_;
  arr path_;
  mlr::Array< ExtensibleKOMO::ptr > komoPathProblems_;

  //-- path opt2
  double pathCost2_, pathConstraints2_;
  bool pathFeasible2_;
  arr path2_;
  mlr::Array< ExtensibleKOMO::ptr > komoPathProblems2_;
  mlr::Array< WorldL > path2Configurations_;

  //--
  int id_;
};

//===========================================================================/*
//class PartiallyObservableNode;

//typedef mlr::Array< PartiallyObservableNode* > PartiallyObservableNodeL;

//class PartiallyObservableNode{
//  friend ostream& operator<<(ostream& os, const PartiallyObservableNode& n);

//public:
//  /// root node init
//  PartiallyObservableNode(mlr::KinematicWorld& kin, mlr::Array< std::shared_ptr< FOL_World > > fols, const KOMOFactory & komoFactory );

//  /// child node creation
//  PartiallyObservableNode(PartiallyObservableNode *parent, uint a );


//  PartiallyObservableNodeL children() const;

//  //- accessors
//  bool isTerminal() const { return getFirst()->isTerminal; }
//  uint poseCount()  const { return getFirst()->poseCount; }
//  bool poseFeasible() const { return getFirst()->poseFeasible; }
//  bool seqFeasible() const { return getFirst()->seqFeasible; }
//  bool pathFeasible() const { return getFirst()->pathFeasible; }
//  MCStatistics * mcStats() const { return getFirst()->mcStats; }
//  std::shared_ptr<ExtensibleKOMO> komoPoseProblem() const { return getFirst()->komoPoseProblem; }
//  std::shared_ptr<ExtensibleKOMO> komoSeqProblem() const { return getFirst()->komoSeqProblem; }
//  std::shared_ptr<ExtensibleKOMO> komoPathProblem() const { return getFirst()->komoPathProblem; }
//  bool& inFringe1() const { return getFirst()->inFringe1; } // also setter
//  bool& inFringe2() const { return getFirst()->inFringe2; } // also setter
//  uint graphIndex() const { return getFirst()->graphIndex; }
//  bool isExpanded() const { return getFirst()->isExpanded; }
//  PartiallyObservableNode * parent() const{ return getFirst()->parent->pobNode; }
//  FOL_World::Handle decision() const { return getFirst()->decision; }
//  double symCost() const { return getFirst()->symCost; }
//  double poseCost() const { return getFirst()->poseCost; }
//  double seqCost() const { return getFirst()->seqCost; }
//  double pathCost() const { return getFirst()->pathCost; }
//  uint seqCount() const { return getFirst()->seqCount; }
//  arr pose() const { return getFirst()->pose; }
//  arr seq() const { return getFirst()->seq; }
//  arr path() const { return getFirst()->path; }

//  //- computations on the node
//  void expand();           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
//  arr generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions);
//  void addMCRollouts(uint num,int stepAbort);
//  void solvePoseProblem(); ///< solve the effective pose problem
//  void solveSeqProblem(int verbose=0);  ///< compute a sequence of key poses along the decision path
//  void solvePathProblem(uint microSteps, int verbose=0); ///< compute a full path along the decision path

//  //-- helpers
//  void labelInfeasible(); ///< sets the infeasible label AND removes all children!
//  PartiallyObservableNodeL getTreePath(); ///< return the decision path in terms of a list of nodes (just walking to the root)
//  PartiallyObservableNode* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
//  void getAllChildren(PartiallyObservableNodeL& tree);
//  PartiallyObservableNode *treePolicy_random(); ///< returns leave -- by descending children randomly
//  PartiallyObservableNode *treePolicy_softMax(double temperature);
//  bool recomputeAllFolStates();
//  void recomputeAllMCStats(bool excludeLeafs=true);

//  void checkConsistency();

//  void write(ostream& os=cout, bool recursive=false) const;
//  void getGraph(Graph& G/*, Node *n=NULL*/) { getFirst()->getGraph(G); }
//  Graph getGraph(){ Graph G; getGraph(G); G.checkConsistency(); return G; }
//  void getAll(PartiallyObservableNodeL& L);
//  PartiallyObservableNodeL getAll(){ PartiallyObservableNodeL L; getAll(L); return L; }

//private:
//  ActionNode * getFirst() const { return nodes_.first(); }
//private:
//  //PartiallyObservableNode * root_;
//  //PartiallyObservableNode * parent_;
//  //PartiallyObservableNodeL children_;

//  mlr::Array< ActionNode * > nodes_;
//};

//inline ostream& operator<<(ostream& os, const PartiallyObservableNode& n){ n.getFirst()->write(os); return os; }*/
