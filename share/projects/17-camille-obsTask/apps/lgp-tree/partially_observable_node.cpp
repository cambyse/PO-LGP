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

PartiallyObservableNode::PartiallyObservableNode(mlr::KinematicWorld& kin, mlr::Array< std::shared_ptr< FOL_World > > fols, const KOMOFactory & komoFactory )
{
  for( auto fol : fols )
  {
    nodes_.append( new ActionNode( this, kin, *fol, komoFactory ) );
  }
}

/// child node creation
PartiallyObservableNode::PartiallyObservableNode(PartiallyObservableNode *parent, uint a )
{
  for( auto parent_node : parent->nodes_ )
  {
    parent_node->fol.setState(parent_node->folState, parent_node->s);
    auto actions = parent_node->fol.get_actions();
    nodes_.append( new ActionNode( this, parent_node, actions[a] ) );
  }
}

PartiallyObservableNodeL PartiallyObservableNode::children() const
{
  PartiallyObservableNodeL children;
  for( auto c : getFirst()->children )
    children.append( c->pobNode );

  return children;
}

void PartiallyObservableNode::expand()
{
  CHECK(!isExpanded(),"");
  if(isTerminal()) return;
  getFirst()->fol.setState(getFirst()->folState, getFirst()->s);
  auto actions = getFirst()->fol.get_actions();

  for( uint a = 0; a < actions.size(); ++a )
  {
     cout <<"  EXPAND DECISION: " <<*actions[a] <<endl;
     new PartiallyObservableNode( this, a );
  }
//  for(FOL_World::Handle& a:actions){
//    cout <<"  EXPAND DECISION: " <<*a <<endl;
//    new PartiallyObservableNode(this, a);
//  }
  if(!getFirst()->children.N)
    getFirst()->isTerminal=true;
  getFirst()->isExpanded=true;
}

arr PartiallyObservableNode::generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions)
{
  return getFirst()->generateRootMCRollouts( num, stepAbort, prefixDecisions );
}

void PartiallyObservableNode::addMCRollouts(uint num,int stepAbort)
{
  getFirst()->addMCRollouts( num, stepAbort );
}

void PartiallyObservableNode::solvePoseProblem()
{
  getFirst()->solvePoseProblem();
}

void PartiallyObservableNode::solveSeqProblem(int verbose)
{
  getFirst()->solveSeqProblem(verbose);
}

void PartiallyObservableNode::solvePathProblem(uint microSteps, int verbose)
{
  getFirst()->solvePathProblem(microSteps, verbose);
}

//-- helpers
void PartiallyObservableNode::labelInfeasible()
{
  getFirst()->labelInfeasible();
}

PartiallyObservableNodeL PartiallyObservableNode::getTreePath()
{
  PartiallyObservableNodeL path;
  ActionNode *node=getFirst();
  for(;node;){
    path.prepend(node->pobNode);
    node = node->parent;
  }
  return path;
}

PartiallyObservableNode* PartiallyObservableNode::getRoot()
{
  return getFirst()->getRoot()->pobNode;
}

PartiallyObservableNode * PartiallyObservableNode::treePolicy_random()
{
  ActionNode* n = getFirst()->treePolicy_random();
  if( n )
    return n->pobNode;
  else
    return nullptr;
}

//PartiallyObservableNode *treePolicy_softMax(double temperature);
bool PartiallyObservableNode::recomputeAllFolStates()
{
  return getFirst()->recomputeAllFolStates();
}

void PartiallyObservableNode::recomputeAllMCStats(bool excludeLeafs)
{
  getFirst()->recomputeAllMCStats(excludeLeafs);
}

void PartiallyObservableNode::checkConsistency()
{
  getFirst()->checkConsistency();
}

void PartiallyObservableNode::write(ostream& os, bool recursive) const
{
  getFirst()->write(os, recursive);
}

void PartiallyObservableNode::getAll(PartiallyObservableNodeL& L)
{
  auto list = getFirst()->getAll();
  for( auto n : list )
    L.append(n->pobNode);
}

RUN_ON_INIT_BEGIN(manipulationTree)
PartiallyObservableNodeL::memMove = true;
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
