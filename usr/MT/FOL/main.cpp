#include "fol.h"
#include <Gui/graphview.h>

//===========================================================================

void testPol(){
  KeyValueGraph G;
  FILE("pol.kvg") >>G;

  cout <<G <<endl;

  cout <<"Q=" <<(forwardChaining_propositional(G, G["Q"]) ?"true":"false") <<endl;
}

//===========================================================================

void testFol1(){
  KeyValueGraph G;

  FILE("fol0.kvg") >>G;

//  cout <<"\n-----------------\n" <<G <<"\n-----------------\n" <<endl;

  ItemL consts = G.getItems("Constant");
  Item *s = G["State"];
  Item *r = G["cruiseto"]->kvg()["precond"];
  ItemL vars = G["cruiseto"]->kvg().getItems("Var");

  cout <<"symbols = " <<G.getItems("Symbol") <<endl;

  cout <<"state = " <<*s <<"\nrule=" <<*r <<endl;


  Item *sub = new Item_typed<KeyValueGraph>(G, new KeyValueGraph);
  new Item_typed<bool>(sub->kvg(), STRINGS(), {vars(0), consts(0)});
  new Item_typed<bool>(sub->kvg(), STRINGS(), {vars(1), consts(2)});
}

//===========================================================================

void testFol2(){
  KeyValueGraph G;

  FILE("fol.kvg") >>G;

//  cout <<"\n-----------------\n" <<G <<"\n-----------------\n" <<endl;

  ItemL consts = G.getItems("Constant");
  ItemL rules = G.getItems("Rule");
  ItemL state = getLiteralsOfScope(G);

//  cout <<"consts = " <<consts <<endl;
//  cout <<"rules = " <<rules <<endl;
  cout <<"INIT STATE = " <<GRAPH(state) <<endl;

//  Graph& rule = rules(3)->kvg();
//  cout <<"rule = " <<rule <<endl;
//  removeInfeasible(rule(0), consts, rule(1), G);
//  for(Item* rule:rules)
//    getSubstitutions(rule->kvg(), state);

  Item *query=G["Query"]->kvg()(0);
  forwardChaining_FOL(G, query);
}

//===========================================================================

void testFol3(){
  KeyValueGraph G;
  FILE("fol.kvg") >>G;

  GraphView view(G);
  view.watch();

}

//===========================================================================

void testFol4(){
  KeyValueGraph G;

  FILE("boxes.kvg") >>G;

  ItemL rules = G.getItems("Rule");
  ItemL constants = G.getItems("Constant");
  ItemL state = getLiteralsOfScope(G);

  for(Item* rule:rules){
    cout <<"*** RULE: " <<*rule <<endl;
    cout <<  "Substitutions:" <<endl;
    ItemL subs = getSubstitutions(rule->kvg(), state, constants);
  }
}

//===========================================================================

void testMonteCarlo(){
  KeyValueGraph Gorig;
  FILE("boxes.kvg") >>Gorig;
  MT::rnd.seed(3);
  uint verbose=1;

  for(uint k=0;k<500;k++){
    KeyValueGraph G = Gorig;
    G.checkConsistency();
    ItemL rules = G.getItems("Rule");
    ItemL constants = G.getItems("Constant");
    Graph& terminal = G.getItem("terminal")->kvg();

    for(uint h=0;h<100;h++){
      if(verbose>2) cout <<"****************** " <<k <<" MonteCarlo rollout step " <<h <<endl;

      ItemL state = getLiteralsOfScope(G);
      if(verbose>2){ cout <<"*** state = "; listWrite(state, cout); cout<<endl; }

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

      //-- pick a random decision
      uint deci = MT::rnd(decisions.N);
      std::pair<Item*, ItemL>& d = decisions(deci);
      if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl; }

      Item *effect = d.first->kvg().last();
      if(verbose>2) cout <<"*** applying" <<*effect <<endl;
      applyEffectLiterals(G, effect, d.second, &d.first->kvg());

      //-- test the terminal state
      if(checkAllMatchesInScope(terminal, &G)){
        if(verbose>0) cout <<"************* TERMINAL STATE FOUND (h=" <<h <<") ************" <<endl;
        state = getLiteralsOfScope(G);
        if(verbose>1){ cout <<"*** FINAL STATE = "; listWrite(state, cout); cout<<endl; }
        break;
      }
    }
  }
}

//===========================================================================


int main(int argn, char** argv){
//  testPol();
//  testFol1();
//  testFol2();
//  testFol3();
//  testFol4();
  testMonteCarlo();
}
