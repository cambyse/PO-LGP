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
#include "environment.h"
#include <Core/graph.h>
#include <Algo/priorityQueue.h>

//===========================================================================

struct MBTS;
struct MBTS_Node;
typedef mlr::Array<MBTS_Node*> MBTS_NodeL;

//===========================================================================

struct MBTS_Node{
  MBTS& MBTS;
  MCTS_Environment& world;
  MCTS_Environment::Handle action;
  MCTS_Environment::Handle state;
  MCTS_Environment::TransitionReturn ret;

  MBTS_Node *parent;
  mlr::Array<MBTS_Node*> children;

  uint d;      ///< decision depth of this node
  double time; ///< real time

  arr g; ///< cost-so-far for each level
  arr h; ///< cost-to-go heuristic for each level

  boolA isEvaluated; ///< for each level
  boolA isInfeasible; ///< for each level
  bool isTerminal=false; ///< for logic level only

  /// root node init
  MBTS_Node(MBTS& MBTS, MCTS_Environment& world);

  /// child node creation
  MBTS_Node(MBTS_Node* parent, const MCTS_Environment::Handle& a);

  ~MBTS_Node(){ NIY; }

  //- computations on the node
  void expand();           ///< expand this node (symbolically: compute possible decisions and add their effect nodes)

  virtual void evaluate(int level=-1){ NIY; }

  //-- helpers
  void labelInfeasible(); ///< sets the infeasible label AND removes all children!
  MBTS_NodeL getTreePath(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  MBTS_Node* getRoot(); ///< return the decision path in terms of a list of nodes (just walking to the root)
  void getAllChildren(MBTS_NodeL& tree);

  bool recomputeAllFolStates();
  void recomputeAllMCStats(bool excludeLeafs=true);

  void checkConsistency();

  void write(ostream& os=cout, bool recursive=false) const;
  void getGraph(Graph& G, Node *n=NULL);
  Graph getGraph(){ Graph G; getGraph(G, NULL); G.checkConsistency(); return G; }

  void getAll(MBTS_NodeL& L);
  MBTS_NodeL getAll(){ MBTS_NodeL L; getAll(L); return L; }
};
stdOutPipe(MBTS_Node)

//===========================================================================

struct MBTS_Heuristic{
  struct Return {
    double g;
    double h;
    bool terminal;
    bool feasible;
  };

  virtual ~MBTS_Heuristic(){}
  virtual Return evaluate(MBTS_Node* n, int level);
};

//===========================================================================

struct MBTS{
  MBTS_Node *root;
  mlr::Array<PriorityQueue<MBTS_Node*>> queue; //for each level
  MBTS_Heuristic& heuristic;

  mlr::Array<MBTS_Node*> solutions;
  uint size, depth;

  MBTS(MCTS_Environment& world, MBTS_Heuristic& heuristic, uint L);

  bool step(int level);
  void run();

  void reportQueue();
};

//===========================================================================

