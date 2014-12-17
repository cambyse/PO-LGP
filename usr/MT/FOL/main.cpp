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
  KeyValueGraph G;

  FILE("boxes.kvg") >>G;

  ItemL rules = G.getItems("Rule");
  ItemL constants = G.getItems("Constant");

  MT::rnd.seed(3);

  for(uint h=0;h<20;h++){
    cout <<"****************** MonteCarlo rollout step " <<h <<endl;

    ItemL state = getLiteralsOfScope(G);
    cout <<"*** state = "; listWrite(state, cout); cout<<endl;

    //-- get all possible decisions
    MT::Array<std::pair<Item*, ItemL> > decisions; //tuples of rule and substitution
    for(Item* rule:rules){
//      cout <<"*** RULE: " <<*rule <<endl;
//      cout <<  "Substitutions:" <<endl;
      ItemL subs = getRuleSubstitutions(rule, state, constants, false);
      for(uint s=0;s<subs.d0;s++){
        decisions.append(std::pair<Item*, ItemL>(rule, subs[s]));
      }
    }

    cout <<"*** # possible decisions: " <<decisions.N <<endl;
//    for(auto d:decisions){
//      cout <<"rule " <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl;
//    }

    //-- pick a random decision
    std::pair<Item*, ItemL>& d = decisions(MT::rnd(decisions.N));
    cout <<"*** decision = " <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl;

    Item *effect = d.first->kvg().last();
    cout <<"*** applying" <<*effect <<endl;
    applyEffectLiterals(G, effect, d.second, &d.first->kvg());

//    state = getLiteralsOfScope(G);
//    cout <<"*** new state = "; listWrite(state, cout); cout<<endl;

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
