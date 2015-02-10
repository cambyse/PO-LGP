//#include <pr2/actionMachine.h>
//#include "manipSim.h"
#include <Ors/ors.h>
#include <FOL/fol.h>


void runMonteCarlo(Graph& G){
//  MT::rnd.seed(3);
  uint verbose=3;

  G.checkConsistency();
  //    Item *Terminate_keyword = G["Terminate"];
  ItemL rules = G.getItems("Rule");
  ItemL constants = G.getItems("Object");
  Graph& actionSequence = G["actionSequence"]->kvg();
  Item *papSymbol = G["pap"];
  //    Graph& terminal = G.getItem("terminal")->kvg();

  for(uint h=0;h<100;h++){
    if(verbose>2) cout <<"****************** MonteCarlo rollout step " <<h <<endl;

    ItemL state = getLiteralsOfScope(G);
    if(verbose>2){ cout <<"*** state = "; listWrite(state, cout); cout<<endl; }
    if(verbose>2){ cout <<"*** actionSequence = "; actionSequence.write(cout," ","{}"); cout<<endl; }

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
        if(verbose>1) cout <<"*** NO DECISIONS LEFT" <<endl;
        return;
      }else{
        //-- pick a random decision
        uint deci = MT::rnd(decisions.N);
        std::pair<Item*, ItemL>& d = decisions(deci);
        if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl; }

        Item *effect = d.first->kvg().last();
        if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(d.second, cout); cout <<endl; }
        applyEffectLiterals(G, effect, d.second, &d.first->kvg());

        //-- append it to store the decision
        actionSequence.append(STRINGS_0(), cat({papSymbol}, d.second), new bool(true), true);
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
