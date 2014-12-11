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

int main(int argn, char** argv){
//  testPol();
//  testFol1();
  testFol2();
//  testFol3();
}
