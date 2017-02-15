/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
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

struct ActionNode;
struct PlainMC;
struct MCStatistics;
typedef mlr::Array<ActionNode*> ActionNodeL;

extern uint COUNT_kin, COUNT_evals, COUNT_poseOpt, COUNT_seqOpt, COUNT_pathOpt;

//=====ExtensibleKOMO==============================================

class ExtensibleKOMO : public KOMO
{
  typedef std::function<void( double, const Graph& facts, Node *n, KOMO &, int verbose )> SymbolGrounder;

public:
  ExtensibleKOMO();

  void registerTask( const mlr::String & type, const SymbolGrounder & grounder );
  void groundTasks( double phase, const Graph& facts, int verbose=0 );

private:
  std::map< mlr::String, SymbolGrounder > tasks_;
};

//=====ExtensibleKOMO==============================================

class KOMOFactory
{
  typedef std::function<void( double, const Graph& facts, Node *n, KOMO &, int verbose )> SymbolGrounder;

public:
  void registerTask( const mlr::String & type, const SymbolGrounder & grounder );
  std::shared_ptr< ExtensibleKOMO > createKomo() const;
private:
  std::map< mlr::String, SymbolGrounder > tasks_;
};

//=============done==============================================================

struct PartiallyObservableNode{
  mlr::Array< ActionNode* > nodes_;
  double pHistory_;
  arr bs_;  // initial belief state
};

struct ActionNode{
  ActionNode *parent;
  mlr::Array<ActionNode*> children; ///< all reacheable children
  uint s;               ///< decision depth/step of this node
  double time;          ///< real time
  uint graphIndex=0;

  //-- info on the state and action this node represents
  FOL_World& fol; ///< the symbolic KB (all Graphs below are subgraphs of this large KB)      ///DOOMED
  mlr::Array< std::shared_ptr<FOL_World> > & folWorlds_;
  double pHistory_;
  arr bs_;  // initial belief state

  FOL_World::Handle decision; ///< the decision that led to this node                         ///DOOMED
  //pobs//FOL_World::Handle observation; ///< the observation that led to this node
  Graph *folState;    ///< the symbolic state after the decision                              ///DOOMED
  mlr::Array< std::shared_ptr<Graph> > folStates_; ///< the array of symbolic state after the decision //and observation
  Node  *folDecision; ///< the predicate in the folState that represents the decision         ///DOOMED
  std::size_t actionId; ///< the action hash that led to this node
  //pobs//Node  *folObservation; ///< the predicate in the folState that represents the decision
  double folReward;   ///< the reward collected with this transition step
  Graph *folAddToState; ///< facts that are added to the state /after/ the fol.transition, e.g., infeasibility predicates

  //-- kinematics: the kinematic structure of the world after the decision path
  const mlr::KinematicWorld& startKinematics; ///< initial start state kinematics
  mlr::KinematicWorld effKinematics; ///< the effective kinematics (computed from kinematics and symbolic state)

  bool isExpanded=false;
  bool hasEffKinematics=false;
  bool isInfeasible=false;
  bool isTerminal=false;

  //-- specs and results of the three optimization problems
  //PlainMC *rootMC;    ///< monte carlo engine
  mlr::Array< std::shared_ptr<PlainMC> > rootMcEngines_;

  MCStatistics *mcStats;  ///< statitics (list of the reward of each rollout)
  std::shared_ptr<ExtensibleKOMO> komoPoseProblem, komoSeqProblem, komoPathProblem; ///< trajectory optimization engines
  arr pose, seq, path;    ///< current best solutions (concatenation of x vectors)
  uint mcCount, poseCount, seqCount, pathCount; ///< total number of mc, pose opt, seq opt, path opt run so far
  double symCost, poseCost, poseConstraints, seqCost, seqConstraints, pathCost, pathConstraints; ///< costs of the best solution so far
  bool symTerminal, poseFeasible, seqFeasible, pathFeasible;

  bool inFringe1, inFringe2; ///< used only when generating reports

  /// root node init
  ActionNode(mlr::KinematicWorld& kin, FOL_World & fol, mlr::Array< std::shared_ptr<FOL_World> > & fols, const arr & bs, const KOMOFactory & komoFactory );

  /// child node creation
  //ActionNode(ActionNode *parent, FOL_World::Handle& a, const KOMOFactory & komoFactory );
  ActionNode(ActionNode *parent, double pHistory, const arr & beliefState/*, Graph * stateBeforeAction*/, std::size_t a, const KOMOFactory & komoFactory );

  //- computations on the node
  void expand();           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)
  arr generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions);
  void addMCRollouts(uint num,int stepAbort);
  void solvePoseProblem(); ///< solve the effective pose problem
  void solveSeqProblem(int verbose=0);  ///< compute a sequence of key poses along the decision path
  void solvePathProblem(uint microSteps, int verbose=0); ///< compute a full path along the decision path

  //-- helpers
  void labelInfeasible(); ///< sets the infeasible label AND removes all children!
  ActionNodeL getTreePath(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  ActionNode* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  void getAllChildren(ActionNodeL& tree);
  ActionNode *treePolicy_random(); ///< returns leave -- by descending children randomly
  ActionNode *treePolicy_softMax(double temperature);
  bool recomputeAllFolStates();
  void recomputeAllMCStats(bool excludeLeafs=true);

  void checkConsistency();

  void write(ostream& os=cout, bool recursive=false) const;
  void getGraph(Graph& G, Node *n=NULL);
  Graph getGraph(){ Graph G; getGraph(G, NULL); G.checkConsistency(); return G; }
  void getAll(ActionNodeL& L);
  ActionNodeL getAll(){ ActionNodeL L; getAll(L); return L; }

private:
  const KOMOFactory & komoFactory_;
};

inline ostream& operator<<(ostream& os, const ActionNode& n){ n.write(os); return os; }

//===========================================================================

