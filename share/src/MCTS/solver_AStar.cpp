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


#include "solver_AStar.h"

AStar_Node::AStar_Node(AStar &astar, MCTS_Environment& world)
  : astar(astar), world(world), parent(NULL), d(0), time(0.){
  astar.size++;
  //this is the root node!
  world.reset_state();
  state = world.get_stateCopy();
//  folState = fol.createStateCopy();
}

AStar_Node::AStar_Node(AStar_Node* parent, const MCTS_Environment::Handle& a)
  : astar(parent->astar), world(parent->world), action(a), parent(parent), d(parent->d+1){
  astar.size++;
  if(d>astar.depth) astar.depth=d;
  parent->children.append(this);
  world.set_state(parent->state);
  CHECK(a,"giving a 'NULL' shared pointer??");
  ret = world.transition(action);
  state = world.get_stateCopy();
  time = parent->time + ret.duration;
  isTerminal = world.is_terminal_state();
  g = parent->g + ret.reward; //cost-so-far
  h = 0.; //heuristic
}

void AStar_Node::expand(){
  CHECK(!isExpanded && !children.N,"");
  if(isTerminal) return;
//  FILE("z.1") <<world <<endl;
  world.set_state(state);
//  FILE("z.2") <<world <<endl;
  auto actions = world.get_actions();
  for(const MCTS_Environment::Handle& a:actions){
    new AStar_Node(this, a);
  }
  isExpanded=true;
}

AStar_NodeL AStar_Node::getTreePath(){
  AStar_NodeL path;
  AStar_Node *node=this;
  for(;node;){
      path.prepend(node);
      node = node->parent;
  }
  return path;
}

void AStar_Node::getGraph(Graph& G, Node* n) {
  if(!n){
    n = G.newNode<bool>({"a:<ROOT>"}, NodeL(), true);
  }else{
    n = G.newNode<bool>({STRING("a:"<<*action)}, {n}, true);
  }
  n->keys.append(STRING("d:" <<d <<" t:" <<time <<' '));
  n->keys.append(STRING("f:" <<g+h <<" g:" <<g <<" h:" <<h));
//  if(mcStats && mcStats->n) n->keys.append(STRING("MC best:" <<mcStats->X.first() <<" n:" <<mcStats->n));
//  n->keys.append(STRING("sym  #" <<mcCount <<" f:" <<symCost <<" terminal:" <<isTerminal));
//  n->keys.append(STRING("pose #" <<poseCount <<" f:" <<poseCost <<" g:" <<poseConstraints <<" feasible:" <<poseFeasible));
//  n->keys.append(STRING("seq  #" <<seqCount <<" f:" <<seqCost <<" g:" <<seqConstraints <<" feasible:" <<seqFeasible));
//  n->keys.append(STRING("path #" <<pathCount <<" f:" <<pathCost <<" g:" <<pathConstraints <<" feasible:" <<pathFeasible));
//  if(folAddToState) n->keys.append(STRING("symAdd:" <<*folAddToState));

  G.getRenderingInfo(n).dotstyle="shape=box";
  if(isInfeasible){
    if(isTerminal)  G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=violet";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=red";
  }else if(isTerminal){
//    if(seqCount || pathCount) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=cyan";
//    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=blue";
  }else{
//    if(poseCount || seqCount || pathCount) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=green";
  }
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
//  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";

//  n->keys.append(STRING("reward:" <<effPoseReward));
  for(AStar_Node *ch:children) ch->getGraph(G, n);
}

void AStar_Node::getAll(AStar_NodeL& L){
  L.append(this);
  for(AStar_Node *ch:children) ch->getAll(L);
}

void AStar_Node::write(ostream& os, bool recursive) const{
  if(action) os <<" a= " <<*action;
  else os <<" a=<ROOT>";
  cout <<"d:" <<d <<" t:" <<time <<" f:" <<g+h <<" g:" <<g <<" h:" <<h <<endl;
  if(recursive) for(AStar_Node *n:children) n->write(os);
}

//===========================================================================

AStar::AStar(MCTS_Environment& world) : root(NULL), size(0), depth(0){
  root = new AStar_Node(*this, world);
  queue.add(0., root);
}

bool AStar::step(){
  if(!queue.N){
    LOG(-1) <<"AStar: queue is empty -> failure?";
    return false;
  }
  auto next =  queue.pop();
  if(next->isTerminal){
    solutions.append(next);
    return true;
  }
  next->expand();
  for(AStar_Node* ch:next->children){
    queue.add(- ch->g - ch->h, ch, true);
  }
  return false;
}

void AStar::reportQueue(){
  cout <<"AStar QUEUE:" <<endl;
  for(const PriorityQueueEntry<AStar_Node*>& n:queue){
    cout <<"p=" <<n.p <<" f=" <<n.x->g+n.x->h <<" g=" <<n.x->g <<" h=" <<n.x->h <<" d=" <<n.x->d <<" a=" <<*n.x->action <<endl;
  }
}

RUN_ON_INIT_BEGIN(manipulationTree)
AStar_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
