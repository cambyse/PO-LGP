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

#include <Kin/kin.h>
#include <Logic/fol_mcts_world.h>
#include <LGP/LGP.h>
#include <Logic/fol.h>
#include <Motion/komo.h>
#include "komo_factory.h"
#include "action_node.h"

struct ActionNode;
struct PlainMC;
struct MCStatistics;
typedef mlr::Array<ActionNode*> ActionNodeL;

extern uint COUNT_kin, COUNT_evals, COUNT_poseOpt, COUNT_seqOpt, COUNT_pathOpt;

//===========================================================================
class PartiallyObservableNode;

typedef mlr::Array< PartiallyObservableNode* > PartiallyObservableNodeL;

class PartiallyObservableNode{
  friend ostream& operator<<(ostream& os, const PartiallyObservableNode& n);

public:
  /// root node init
  PartiallyObservableNode(mlr::KinematicWorld& kin, FOL_World& fol, const KOMOFactory & komoFactory );

  /// child node creation
  PartiallyObservableNode(PartiallyObservableNode *parent, FOL_World::Handle& a, const KOMOFactory & komoFactory );


  PartiallyObservableNodeL children() const;

  //- accessors
  bool isTerminal() const { return node_->isTerminal; }
  uint poseCount()  const { return node_->poseCount; }
  bool poseFeasible() const { return node_->poseFeasible; }
  bool seqFeasible() const { return node_->seqFeasible; }
  bool pathFeasible() const { return node_->pathFeasible; }
  MCStatistics * mcStats() const { return node_->mcStats; }
  std::shared_ptr<ExtensibleKOMO> komoPoseProblem() const { return node_->komoPoseProblem; }
  std::shared_ptr<ExtensibleKOMO> komoSeqProblem() const { return node_->komoSeqProblem; }
  std::shared_ptr<ExtensibleKOMO> komoPathProblem() const { return node_->komoPathProblem; }
  bool& inFringe1() const { return node_->inFringe1; } // also setter
  bool& inFringe2() const { return node_->inFringe2; } // also setter
  uint graphIndex() const { return node_->graphIndex; }
  bool isExpanded() const { return node_->isExpanded; }
  PartiallyObservableNode * parent() const{ return node_->parent->pobNode; }
  FOL_World::Handle decision() const { return node_->decision; }
  double symCost() const { return node_->symCost; }
  double poseCost() const { return node_->poseCost; }
  double seqCost() const { return node_->seqCost; }
  double pathCost() const { return node_->pathCost; }
  uint seqCount() const { return node_->seqCount; }
  arr pose() const { return node_->pose; }
  arr seq() const { return node_->seq; }
  arr path() const { return node_->path; }

  //- computations on the node
  void expand();           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
  arr generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions);
  void addMCRollouts(uint num,int stepAbort);
  void solvePoseProblem(); ///< solve the effective pose problem
  void solveSeqProblem(int verbose=0);  ///< compute a sequence of key poses along the decision path
  void solvePathProblem(uint microSteps, int verbose=0); ///< compute a full path along the decision path

  //-- helpers
  void labelInfeasible(); ///< sets the infeasible label AND removes all children!
  PartiallyObservableNodeL getTreePath(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  PartiallyObservableNode* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  void getAllChildren(PartiallyObservableNodeL& tree);
  PartiallyObservableNode *treePolicy_random(); ///< returns leave -- by descending children randomly
  PartiallyObservableNode *treePolicy_softMax(double temperature);
  bool recomputeAllFolStates();
  void recomputeAllMCStats(bool excludeLeafs=true);

  void checkConsistency();

  void write(ostream& os=cout, bool recursive=false) const;
  void getGraph(Graph& G/*, Node *n=NULL*/) { node_->getGraph(G); }
  Graph getGraph(){ Graph G; getGraph(G); G.checkConsistency(); return G; }
  void getAll(PartiallyObservableNodeL& L);
  PartiallyObservableNodeL getAll(){ PartiallyObservableNodeL L; getAll(L); return L; }

private:
  //PartiallyObservableNode * root_;
  //PartiallyObservableNode * parent_;
  //PartiallyObservableNodeL children_;

  ActionNode * node_;

};

inline ostream& operator<<(ostream& os, const PartiallyObservableNode& n){ n.node_->write(os); return os; }
