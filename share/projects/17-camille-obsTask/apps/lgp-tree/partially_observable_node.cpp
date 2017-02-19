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


#include "partially_observable_node.h"
#include <MCTS/solver_PlainMC.h>

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) x

//===========================================================================

PartiallyObservableNode::PartiallyObservableNode(mlr::KinematicWorld& kin, FOL_World& fol, const KOMOFactory & komoFactory )
  : node_( new ActionNode( this, kin, fol, komoFactory ) )
{

}

/// child node creation
PartiallyObservableNode::PartiallyObservableNode(PartiallyObservableNode *parent, FOL_World::Handle& a, const KOMOFactory & komoFactory )
  : node_( new ActionNode( this, parent->node_, a, komoFactory ) )
{

}

PartiallyObservableNodeL PartiallyObservableNode::children() const
{
  PartiallyObservableNodeL children;
  for( auto c : node_->children )
    children.append( c->pobNode );

  return children;
}

void PartiallyObservableNode::expand()
{
  CHECK(!isExpanded(),"");
  if(isTerminal()) return;
  node_->fol.setState(node_->folState, node_->s);
  auto actions = node_->fol.get_actions();
  for(FOL_World::Handle& a:actions){
//    cout <<"  EXPAND DECISION: " <<*a <<endl;
    new PartiallyObservableNode(this, a, node_->komoFactory_);
  }
  if(!node_->children.N) node_->isTerminal=true;
  node_->isExpanded=true;
}

arr PartiallyObservableNode::generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions)
{
  return node_->generateRootMCRollouts( num, stepAbort, prefixDecisions );
}

void PartiallyObservableNode::addMCRollouts(uint num,int stepAbort)
{
  node_->addMCRollouts( num, stepAbort );
}

void PartiallyObservableNode::solvePoseProblem()
{
  node_->solvePoseProblem();
}

void PartiallyObservableNode::solveSeqProblem(int verbose)
{
  node_->solveSeqProblem(verbose);
}

void PartiallyObservableNode::solvePathProblem(uint microSteps, int verbose)
{
  node_->solvePathProblem(microSteps, verbose);
}

//-- helpers
void PartiallyObservableNode::labelInfeasible()
{
  node_->labelInfeasible();
}

PartiallyObservableNodeL PartiallyObservableNode::getTreePath()
{
  PartiallyObservableNodeL path;
  ActionNode *node=node_;
  for(;node;){
    path.prepend(node->pobNode);
    node = node->parent;
  }
  return path;
}

PartiallyObservableNode* PartiallyObservableNode::getRoot()
{
  return node_->getRoot()->pobNode;
}

void getAllChildren(PartiallyObservableNodeL& tree);
PartiallyObservableNode * PartiallyObservableNode::treePolicy_random()
{
  ActionNode* n = node_->treePolicy_random();
  if( n )
    return n->pobNode;
  else
    return nullptr;
}

//PartiallyObservableNode *treePolicy_softMax(double temperature);
bool PartiallyObservableNode::recomputeAllFolStates()
{
  return node_->recomputeAllFolStates();
}

void PartiallyObservableNode::recomputeAllMCStats(bool excludeLeafs)
{
  node_->recomputeAllMCStats(excludeLeafs);
}

void PartiallyObservableNode::checkConsistency()
{
  node_->checkConsistency();
}

void PartiallyObservableNode::write(ostream& os, bool recursive) const
{
  node_->write(os, recursive);
}

void PartiallyObservableNode::getAll(PartiallyObservableNodeL& L)
{
  auto list = node_->getAll();
  for( auto n : list )
    L.append(n->pobNode);
}

RUN_ON_INIT_BEGIN(manipulationTree)
PartiallyObservableNodeL::memMove = true;
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
