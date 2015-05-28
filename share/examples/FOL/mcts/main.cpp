#include <FOL/fol.h>
#include <Gui/graphview.h>
#include <MCTS/solver_marc.h>

//===========================================================================

void testMonteCarlo(){
  Graph Gorig;
  FILE("boxes.kvg") >>Gorig;
  MT::rnd.seed(3);
  uint verbose=3;

  for(uint k=0;k<10;k++){
    Graph KB = Gorig;
    KB.checkConsistency();
    Item *Terminate_keyword = KB["Terminate"];
    Graph& state = KB.getItem("STATE")->kvg();
    ItemL rules = KB.getItems("Rule");
    ItemL constants = KB.getItems("Constant");
    Graph& terminal = KB.getItem("terminal")->kvg();

    for(uint h=0;h<100;h++){
      if(verbose>2) cout <<"****************** " <<k <<" MonteCarlo rollout step " <<h <<endl;

      if(verbose>2){ cout <<"*** state = "; state.write(cout, " "); cout <<endl; }

      bool forceWait=false, decideWait=false;
      if(MT::rnd.uni()<.8){ //normal rule decision
        //-- get all possible decisions
        MT::Array<std::pair<Item*, ItemL> > decisions; //tuples of rule and substitution
        for(Item* rule:rules){
          ItemL subs = getRuleSubstitutions(state, rule, constants, (verbose>4) );
          for(uint s=0;s<subs.d0;s++){
            decisions.append(std::pair<Item*, ItemL>(rule, subs[s]));
          }
        }

        if(verbose>2) cout <<"*** # possible decisions: " <<decisions.N <<endl;
        if(verbose>3) for(auto d:decisions){
          cout <<"rule " <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl;
        }

        if(!decisions.N){
          forceWait=true;
        }else{
          //-- pick a random decision
          uint deci = MT::rnd(decisions.N);
          std::pair<Item*, ItemL>& d = decisions(deci);
          if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl; }

          Item *effect = d.first->kvg().last();
          if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(d.second, cout); cout <<endl; }
          applyEffectLiterals(state, effect->kvg(), d.second, &d.first->kvg());
        }
      }else{
        decideWait=true;
      }

      if(forceWait || decideWait){
        if(verbose>2){ cout <<"*** WAIT decision " <<endl; }

        //-- find minimal wait time
        double w=1e10;
        for(Item *i:state){
          if(i->getValueType()==typeid(double)){
            double wi = *i->getValue<double>();
            if(w>wi) w=wi;
          }
        }

        if(w==1e10){
          if(verbose>2) cout <<"*** not applicable" <<endl;
          if(forceWait){ cout <<"*** STUCK - NO FEASIBLE SOLUTION FOUND" <<endl;  break; }
        }else{
          //-- subtract w from all times and collect all activities with minimal wait time
          ItemL activities;
          for(Item *i:state){
            if(i->getValueType()==typeid(double)){
              double &wi = *i->getValue<double>();
              wi -= w;
              if(fabs(wi)<1e-10) activities.append(i);
            }
          }

          //-- for all these activities call the terminate operator
          for(Item *act:activities){
            Item *predicate = act->parents(0);
            Item *rule = KB.getChild(Terminate_keyword, predicate);
            if(!rule) HALT("No termination rule for '" <<*predicate <<"'");
            Item *effect = rule->kvg().last();
            ItemL vars = getSymbolsOfScope(rule->kvg());
            ItemL subs(vars.N); subs.setZero();
            CHECK(vars.N==act->parents.N-1,"");
            for(uint i=0;i<vars.N;i++) subs(i) = act->parents(i+1);

            if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs, cout); cout <<endl; }
            applyEffectLiterals(state, effect->kvg(), subs, &rule->kvg());
          }
        }
      }

      //-- test the terminal state
      if(allFactsHaveEqualsInScope(state, terminal)){
        if(verbose>0) cout <<"************* TERMINAL STATE FOUND (h=" <<h <<") ************" <<endl;
        if(verbose>1){ cout <<"*** FINAL STATE = "; state.write(cout, " "); cout <<endl; }
        break;
      }
    }
  }
}

//===========================================================================

#include "fol_mcts_world.h"

void testMCTS(){
  FOL_World world("boxes.kvg");
  world.verbose=4;
  MCTS mcts(world);
//  Graph G = mcts.getGraph();
//  GraphView gv(G);
  for(uint k=0;k<100;k++){
    cout <<"******************************************** ROLLOUT " <<k <<endl;
    mcts.addRollout();
//    G = mcts.getGraph();
//    if(!(k%1)) gv.update();
  }
  cout <<mcts.Qfunction() <<endl;

}

//===========================================================================

int main(int argn, char** argv){
//  testMonteCarlo();
  testMCTS();
}
