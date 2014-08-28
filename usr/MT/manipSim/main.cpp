#include <Ors/ors.h>
#include <pr2/actionMachine.h>

void init(){
  new Symbol("rigid", 2);
  new Symbol("2DtransPhi", 2);
}

//===========================================================================

enum Predicate{ free=0, rigid, trans2DPhi };

struct Relation{
  Predicate p;
  uint i,j;
};

typedef MT::Array<Relation*> RelationL;

struct RelationalState{
  uint N; ///< numberof objects
  RelationL R;
};

//===========================================================================

void sample(){
  ors::KinematicWorld W("model.kvg");
  reportExistingSymbols();

  //-- fwd expansion
  SearchNodeL T;
  SearchNode *goal=NULL;

  for(uint k=0;k<100;k++){
    SearchNode n = T(k);
    ActionL A = n.getFeasibleActions();
    for(Action *a:A){
      State s = n.getState();
      s = s.apply(a);
      s.expandReachable();
      n.append(s, T);
      if(checkGoalIsFeasible(s)){
        goal = T.last();
        break;
      }
    }
  }

  //backtracking
  SearchNodeL plan = backtrack<SearchNode>(T,goal);
  for(SearchNode *n:plan){
    cout <<"pre-action=" <<n->getPreAction() <<endl;
    cout <<"state=" <<n->getState() <<endl;
  }

}

//===========================================================================

int main(int argc,char **argv){
  init();
  sample();
  return 0;
}
