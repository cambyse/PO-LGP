//#include <RosCom/actionMachine.h>
#include "manipSim.h"
#include <Kin/kin.h>

mlr::KinematicWorld* world=NULL;

void OrsGraph2RelationalGraph(Graph& G, mlr::KinematicWorld& W){
  G.clear();

  //do this first to ensure they have the same indexing
  for(mlr::Body *b:world->bodies){
    G.newNode<mlr::Body>({"body", b->name}, b);
  }

  for(mlr::Body *b:world->bodies){
    G.newNode<mlr::Transformation>({"pose"}, ARRAY(G(b->index)), new mlr::Transformation(b->X));
//    if(b->ats["ctrlable"]) G.newNode<bool>({"controllable"}, ARRAY(G(b->index)), NULL);
    if(b->ats["canGrasp"]) G.newNode<bool>({"canGrasp"}, ARRAY(G(b->index)), NULL);
    if(b->ats["fixed"])    G.newNode<bool>({"fixed"}, ARRAY(G(b->index)), NULL);
  }

  for(mlr::Joint *j:world->joints){
    if(j->type==mlr::JT_rigid)
      G.newNode<bool>({"rigid"}, ARRAY(G(j->from->index), G(j->to->index)), NULL);
    if(j->type==mlr::JT_transXYPhi)
      G.newNode<bool>({"support"}, ARRAY(G(j->from->index), G(j->to->index)), NULL);
  }

}

uint Domain::numObjects(){
  return world->bodies.N;
}

void Domain::getInitialState(State &s){
//  enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_transXY=6, JT_trans3=7, JT_transXYPhi=8, JT_universal=9, JT_rigid=10, JT_quatBall, JT_glue };

//  for(mlr::Body *b:world->bodies){
//    Pose *p = new Pose;
//    p->mean = b->X;
//    p->max = p->mean.pos;
//    p->min = p->mean.pos;
//    p->rotRange.setZero();
//    s.poses.append(p);
//  }

  OrsGraph2RelationalGraph(s.G, *world);
}

//===========================================================================

void sample(){
  mlr::KinematicWorld W("model.kvg");
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

    mlr::wait();

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

int main(int argc,char **argv){
//  rnd.clockSeed();

  sample();
  return 0;
}
