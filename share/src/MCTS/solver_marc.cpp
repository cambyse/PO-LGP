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


#include "solver_marc.h"

void MCTS::addRollout(int stepAbort){
  int step=0;
  MCTS_Node *n = &root;
  world.reset_state();

  //-- tree policy
  double Return_tree=0.;
  while(!world.is_terminal_state() && (stepAbort<0 || step++<stepAbort)){
    if(!n->children.N && !n->N) break; //freshmen -> do not expand
    if(!n->children.N && n->N){ //expand: compute new decisions and add corresponding nodes
      if(verbose>2) cout <<"****************** MCTS: expanding: computing all decisions for current node and adding them as freshmen nodes" <<endl;
      for(const MCTS_Environment::Handle& d:world.get_actions()) new MCTS_Node(n, d); //this adds a bunch of freshmen for all possible decisions
      //n->children.permuteRandomly();
    }else{
      if(verbose>2) cout <<"****************** MCTS: decisions in current node already known" <<endl;
    }
#if 1
    //DEBUG whether decision set is correct
    auto A = world.get_actions();
    CHECK(n->children.N==A.size(),"");
//    for(auto &a:A) cout <<*a <<endl;
//    cout <<endl;
//    for(auto &ch:n->children) cout <<*ch->decision <<endl;
    for(uint i=0;i<n->children.N;i++){
      mlr::String d1,d2;
      d1 <<*n->children(i)->decision;
      d2 <<*A[i];
      CHECK_EQ(d1, d2, "");
    }
#endif
    n = treePolicy(n);
    if(verbose>1) cout <<"****************** MCTS: made tree policy decision" <<endl;
    Return_tree += n->r = world.transition(n->decision).reward;
  }

  //-- rollout
  double Return_rollout=0.;
  while(!world.is_terminal_state() && (stepAbort<0 || step++<stepAbort)){
    if(verbose>1) cout <<"****************** MCTS: random decision" <<endl;
    Return_rollout += world.transition_randomly().reward;
  }

  //  double r = world.get_terminal_reward();
  //  Return_rollout += r;
  if(step>=stepAbort) Return_rollout -= 100.;
  if(verbose>0) cout <<"****************** MCTS: terminal state reached; step=" <<step <<" Return=" <<Return_tree + Return_rollout <<endl;

  //-- backup
  double Return_togo = Return_rollout;
  for(;;){
    if(!n) break;
    n->N++;
    n->R += n->r;   //total immediate reward
    Return_togo += n->r; //add up total return from n to terminal
    n->Q += Return_togo;
    if(n->children.N && n->N>n->children.N){ //propagate bounds
      n->Qup = max( Qfunction(n, +1) );
      n->Qme = max( Qfunction(n,  0) );
      n->Qlo = max( Qfunction(n, -1) );
    }
    n = n->parent;
  }
}

MCTS_Node* MCTS::treePolicy(MCTS_Node* n){
  CHECK(n->children.N, "you should have children!");
  CHECK(n->N, "you should not be a freshman!");
  if(n->N>n->children.N){ //we've visited each child at least once
    arr Q = Qfunction(n, +1);      //optimistic Qfunction
    rndUniform(Q, 0., 1e-3, true); //add noise
    return n->children( argmax( Q ) );
  }
  return n->children( n->N-1 ); //else: visit children by their order
}

arr MCTS::Qfunction(MCTS_Node* n, int optimistic){
  if(!n) n=&root;
  if(!n->children.N) return arr();
  arr Q(n->children.N);
  uint i=0;
  for(MCTS_Node *ch:n->children){ Q(i) = Qvalue(ch, optimistic); i++; }
  return Q;
}

arr MCTS::Qvariance(MCTS_Node* n){
  if(!n) n=&root;
  if(!n->children.N) return arr();
  arr QV(n->children.N);
  uint i=0;
  for(MCTS_Node *ch:n->children){ QV(i) = ch->Qup - ch->Qlo; i++; }
  return QV;
}

void MCTS::reportQ(ostream& os, MCTS_Node* n){
  if(!n) n=&root;
  if(!n->children.N) return;
  uint i=0;
  for(MCTS_Node *ch:n->children){
    os <<'t' <<ch->t <<'N' <<ch->N <<'[' <<ch->Qlo <<',' <<ch->Qme <<',' <<ch->Qup <<']' <<endl;
    i++;
  }
}

void MCTS::reportDecisions(ostream& os, MCTS_Node* n){
  if(!n) n=&root;
  if(!n->children.N) return;
  for(MCTS_Node *ch:n->children){
    os <<'t' <<ch->t <<'N' <<ch->N <<" D=" <<*ch->decision <<endl;
  }
}

uint MCTS::Nnodes(MCTS_Node *n, bool subTree){
  if(!n) n=&root;
  if(!subTree) return n->children.N;
  uint i=1;
  for(MCTS_Node *ch:n->children) i += Nnodes(ch, true);
  return i;
}

double MCTS::Qvalue(MCTS_Node* n, int optimistic){
  if(false && n->children.N && n->N>n->children.N){ //the child is mature and has children itself
    if(optimistic==+1) return n->Qup;
    if(optimistic== 0) return n->Qme;
    if(optimistic==-1) return n->Qlo;
  }else{
    //the child is premature -> use its on-policy return estimates (and UCB)
    double c = beta*sqrt(2.*::log(n->parent?n->parent->N:n->N));
    if(optimistic==+1) return n->Q/n->N + c/sqrt(n->N);
    if(optimistic== 0) return n->Q/n->N;
    if(optimistic==-1) return n->Q/n->N - c/sqrt(n->N);
  }
  HALT("");
  return 0.;
}

void MCTS::writeToGraph(Graph& G, MCTS_Node* n){
  NodeL par;
  if(!n) n=&root; else par.append((Node*)(n->parent->data));
  double q=-10.;  if(n->N) q=n->Q/n->N;
  n->data = G.newNode<double>({STRING("t"<<n->t <<'N' <<n->N <<'[' <<n->Qlo <<',' <<n->Qme <<',' <<n->Qup <<']')}, par, q);
  for(MCTS_Node *c:n->children) writeToGraph(G, c);
}
