//#include <pr2/actionMachine.h>
#include "manipSim.h"
#include <Ors/ors.h>

ors::KinematicWorld* world=NULL;


uint Domain::numObjects(){
  return world->bodies.N;
}

bool Domain::isDirectlyControllable(uint i){
  Item *a = world->bodies(i)->ats["ctrlable"];
  return a!=NULL;
}

void Domain::getInitialState(State &s){
  s.R.clear();
  s.P.clear();
  s.G.clear();
//  enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_transXY=6, JT_trans3=7, JT_transXYPhi=8, JT_universal=9, JT_fixed=10, JT_quatBall, JT_glue };

  for(ors::Body *b:world->bodies){
    Pose *p = new Pose;
    p->mean = b->X;
    p->max = p->mean.pos;
    p->min = p->mean.pos;
    p->rotRange.setZero();
    s.P.append(p);
  }

  for(ors::Joint *j:world->joints){
    if(j->type==ors::JT_fixed) s.R.append(new Relation(rigid, j->from->index, j->to->index));
    if(j->type==ors::JT_transXYPhi) s.R.append(new Relation(trans2DPhi, j->from->index, j->to->index));
  }
}

void Domain::getNewState(State& new_s, const State& s, const Action& a){
  new_s = s;
  new_s.preAction = a;
}

//===========================================================================

void sample(){
  ors::KinematicWorld W("model.kvg");
  world = &W;

  //-- fwd expansion
  SearchNodeL T;
  SearchNode *root=new SearchNode(T);
  SearchNode *goal=NULL;

  cout <<*T(0) <<endl;

  for(uint k=0;k<5;k++){
    SearchNode *n = T(k);
    Action a = n->getRandomFeasibleAction();
//    for(Action *a:A){
    SearchNode *m = new SearchNode(*n, a);
    m->state.expandReachable();
    cout <<*m <<endl;

//      if(checkGoalIsFeasible(s)){
//        goal = T.last();
//        break;
//      }
//    }
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
  sample();
  return 0;
}
