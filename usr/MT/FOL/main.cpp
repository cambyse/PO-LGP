#include <Core/keyValueGraph.h>

void testFol(){
  KeyValueGraph G;

  FILE("fol.kvg") >>G;

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

bool forwardChaining(KeyValueGraph& KB, Item* q){
  KB.checkConsistency();
  uintA count(KB.N);     count=0;
  boolA inferred(KB.N);  inferred=false;
  ItemL clauses = KB.getItems("Clause");
  ItemL agenda;
  for(Item *clause:clauses){
    count(clause->index) = clause->kvg().N;
    if(!count(clause->index)){ //no preconditions -> facts -> add to 'agenda'
      agenda.append(clause->parents(0));
    }
  }
  cout <<count <<endl;

  while(agenda.N){
    Item *s = agenda.popFirst();
    if(!inferred(s->index)){
      inferred(s->index) = true;
      for(Item *child : s->parentOf){ //all objects that involve 's'
        Item *clause = child->container.isItemOfParentKvg; //check if child is a literal in a clause
        if(clause){ //yes: 's' is a literal in a clause
          CHECK(count(clause->index)>0,"");
//          if(count(clause->index)>0){ //I think this is always true...
          count(clause->index)--;
          if(!count(clause->index)){ //are all the preconditions fulfilled?
            Item *newFact = clause->parents(0);
            if(newFact==q) return true;
            agenda.append(newFact);
          }
        }
        cout <<count <<endl;
      }
    }
  }
  return false;
}

void testPol(){
  KeyValueGraph G;
  FILE("pol.kvg") >>G;

  cout <<G <<endl;

  cout <<"Q=" <<(forwardChaining(G, G["Q"]) ?"true":"false") <<endl;
}

int main(int argn, char** argv){
  testPol();
}
