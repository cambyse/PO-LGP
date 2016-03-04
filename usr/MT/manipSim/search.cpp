//#include <pr2/actionMachine.h>
//#include "manipSim.h"
#include <Ors/ors.h>
#include <FOL/fol.h>


void runMonteCarlo(Graph& G){
//  mlr::rnd.seed(3);
  uint verbose=0;

  G.checkConsistency();
  //    Node *Terminate_keyword = G["Terminate"];
  NodeL rules = G.getNodes("Rule");
  NodeL constants = G.getNodes("Object");
  Graph& actionSequence = G["actionSequence"]->graph();
  Node *papSymbol = G["pap"];
  Node *depthSymbol = G["depth"];
  Graph& state = G["STATE"]->graph();
  //    Graph& terminal = G.getNode("terminal")->graph();

  for(uint h=0;h<100;h++){
    if(verbose>2) cout <<"****************** MonteCarlo rollout step " <<h <<endl;

    if(verbose>2){ cout <<"*** state = "; state.write(cout, " ", "{}"); cout<<endl; }
    if(verbose>2){ cout <<"*** actionSequence = "; actionSequence.write(cout," ","{}"); cout<<endl; }

    {
      //-- get all possible decisions
      mlr::Array<std::pair<Node*, NodeL> > decisions; //tuples of rule and substitution
      for(Node* rule:rules){
        //      cout <<"*** RULE: " <<*rule <<endl;
        //      cout <<  "Substitutions:" <<endl;
        NodeL subs = getRuleSubstitutions2(state, rule, (verbose-3) );
        for(uint s=0;s<subs.d0;s++){
          decisions.append(std::pair<Node*, NodeL>(rule, subs[s]));
        }
      }

      if(verbose>2) cout <<"*** # possible decisions: " <<decisions.N <<endl;
      if(verbose>3) for(auto d:decisions){
        cout <<"rule " <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl;
      }

      if(!decisions.N){
        if(verbose>1) cout <<"*** NO DECISIONS LEFT" <<endl;
        return;
      }else{
        //-- pick a random decision
        uint deci = mlr::rnd(decisions.N);
        std::pair<Node*, NodeL>& d = decisions(deci);
        if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl; }

        Node *effect = d.first->graph().last();
        if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(d.second, cout); cout <<endl; }
        applyEffectLiterals(state, effect->graph(), d.second, &d.first->graph());

        //hack: apply depth effect:
        Node *depth0=NULL, *depth1=NULL;
        for(Node *fact:d.second(0)->parentOf) if(&fact->container==&state && fact->parents(0)==depthSymbol){
          depth0=fact; break;
        }
        for(Node *fact:d.second(1)->parentOf) if(&fact->container==&state && fact->parents(0)==depthSymbol){
          depth1=fact; break;
        }
        if(depth0 && depth1){
          depth0->get<double>() = depth1->get<double>() + 1.;
        }

        //-- append it to store the decision
        actionSequence.append({d.first->keys(1)}, cat({papSymbol}, d.second), new bool(true), true);
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
