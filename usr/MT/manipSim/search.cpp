//#include <pr2/actionMachine.h>
#include "manipSim.h"
#include <Ors/ors.h>
#include <FOL/fol.h>

ors::KinematicWorld* world=NULL;

void OrsGraph2RelationalGraph(KeyValueGraph& G, ors::KinematicWorld& W){
  G.clear();

  //do this first to ensure they have the same indexing
  for(ors::Body *b:world->bodies){
    G.append<ors::Body>(STRINGS("body", b->name), b, false);
  }

  for(ors::Body *b:world->bodies){
    G.append<ors::Transformation>(STRINGS("pose"), ARRAY(G(b->index)), new ors::Transformation(b->X), true);
//    if(b->ats["ctrlable"]) G.append<bool>(STRINGS("controllable"), ARRAY(G(b->index)), NULL);
    if(b->ats["canGrasp"]) G.append<bool>(STRINGS("canGrasp"), ARRAY(G(b->index)), NULL, false);
    if(b->ats["fixed"])    G.append<bool>(STRINGS("fixed"), ARRAY(G(b->index)), NULL, false);
  }

  for(ors::Joint *j:world->joints){
    if(j->type==ors::JT_fixed)
      G.append<bool>(STRINGS("rigid"), ARRAY(G(j->from->index), G(j->to->index)), NULL, false);
    if(j->type==ors::JT_transXYPhi)
      G.append<bool>(STRINGS("support"), ARRAY(G(j->from->index), G(j->to->index)), NULL, false);
  }

}

uint Domain::numObjects(){
  return world->bodies.N;
}

void Domain::getInitialState(State &s){
  OrsGraph2RelationalGraph(s.G, *world);
}

//===========================================================================

void sample(){
  ors::KinematicWorld W("model.kvg");
  world = &W;

  //-- fwd expansion
  SearchNodeL T;
  SearchNode *root=new SearchNode(T);
  SearchNode *goal=NULL;

  cout <<"initial state=\n" <<*root <<endl;

  for(uint k=0;k<10;k++){
    SearchNode *n = T(k);
    Action a = n->getRandomFeasibleAction();
    cout <<"random action=" <<a <<endl;
    SearchNode *m = new SearchNode(*n, a);
    m->state.expandReachable();
    cout <<"new state=\n" <<*m <<endl;

    MT::wait();

//      if(checkGoalIsFeasible(s)){
//        goal = T.last();
//        break;
//      }
//    }
  }

  return;
  //backtracking
  SearchNodeL plan = backtrack<SearchNode>(T,goal);
  for(SearchNode *n:plan){
    cout <<"pre-action=" <<n->getPreAction() <<endl;
    cout <<"state=" <<n->getState() <<endl;
  }

}

//===========================================================================

void testMonteCarlo(){
  ors::KinematicWorld world("model.kvg");
  Graph Gorig("manip.kvg");
  MT::rnd.seed(3);
  uint verbose=3;

  for(uint k=0;k<2;k++){
    KeyValueGraph G = Gorig;
    G.checkConsistency();
//    Item *Terminate_keyword = G["Terminate"];
    ItemL rules = G.getItems("Rule");
    ItemL constants = G.getItems("Symbol");
//    Graph& terminal = G.getItem("terminal")->kvg();

    for(uint h=0;h<100;h++){
      if(verbose>2) cout <<"****************** " <<k <<" MonteCarlo rollout step " <<h <<endl;

      ItemL state = getLiteralsOfScope(G);
      if(verbose>2){ cout <<"*** state = "; listWrite(state, cout); cout<<endl; }

      {
        //-- get all possible decisions
        MT::Array<std::pair<Item*, ItemL> > decisions; //tuples of rule and substitution
        for(Item* rule:rules){
          //      cout <<"*** RULE: " <<*rule <<endl;
          //      cout <<  "Substitutions:" <<endl;
          ItemL subs = getRuleSubstitutions(rule, state, constants, (verbose>4) );
          for(uint s=0;s<subs.d0;s++){
            decisions.append(std::pair<Item*, ItemL>(rule, subs[s]));
          }
        }

        if(verbose>2) cout <<"*** # possible decisions: " <<decisions.N <<endl;
        if(verbose>3) for(auto d:decisions){
          cout <<"rule " <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl;
        }

        if(!decisions.N){
          HALT("");
        }else{
          //-- pick a random decision
          uint deci = MT::rnd(decisions.N);
          std::pair<Item*, ItemL>& d = decisions(deci);
          if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl; }

          Item *effect = d.first->kvg().last();
          if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(d.second, cout); cout <<endl; }
          applyEffectLiterals(G, effect, d.second, &d.first->kvg());
        }
      }

      //-- test the terminal state
//      if(checkAllMatchesInScope(terminal, &G)){
//        if(verbose>0) cout <<"************* TERMINAL STATE FOUND (h=" <<h <<") ************" <<endl;
//        state = getLiteralsOfScope(G);
//        if(verbose>1){ cout <<"*** FINAL STATE = "; listWrite(state, cout); cout<<endl; }
//        break;
//      }
    }
  }
}
