//#include <pr2/actionMachine.h>
//#include "manipSim.h"
#include <Ors/ors.h>
#include <Gui/opengl.h>

void sample();

void RelationalGraph2OrsGraph(ors::KinematicWorld& W, const KeyValueGraph& G){
  W.qdim.clear();
  W.q.clear();
  W.qdot.clear();
  listDelete(W.proxies);
  while(W.joints.N) delete W.joints.last();
  W.isLinkTree=false;
  W.checkConsistency();

  for(Item *i:G){

  }

//  //do this first to ensure they have the same indexing
//  for(ors::Body *b:world->bodies){
//    G.append<ors::Body>(STRINGS("body", b->name), b);
//  }

//  for(ors::Body *b:world->bodies){
//    G.append<ors::Transformation>(STRINGS("pose"), ARRAY(G(b->index)), new ors::Transformation(b->X));
////    if(b->ats["ctrlable"]) G.append<bool>(STRINGS("controllable"), ARRAY(G(b->index)), NULL);
//    if(b->ats["canGrasp"]) G.append<bool>(STRINGS("canGrasp"), ARRAY(G(b->index)), NULL);
//    if(b->ats["fixed"])    G.append<bool>(STRINGS("fixed"), ARRAY(G(b->index)), NULL);
//  }

//  for(ors::Joint *j:world->joints){
//    if(j->type==ors::JT_fixed)
//      G.append<bool>(STRINGS("rigid"), ARRAY(G(j->from->index), G(j->to->index)), NULL);
//    if(j->type==ors::JT_transXYPhi)
//      G.append<bool>(STRINGS("support"), ARRAY(G(j->from->index), G(j->to->index)), NULL);
//  }

}

//===========================================================================

void testReachable() {

  KeyValueGraph G;
  ors::KinematicWorld world("model.kvg");

//  G <<FILE("state.kvg");
  RelationalGraph2OrsGraph(world, G);


//  world.checkConsistency();
//  world >>FILE("z.ors");
//  //some optional manipulations
//  world.checkConsistency();
//  world.setShapeNames();
//  world.checkConsistency();
//  world.meldFixedJoints();
//  world.checkConsistency();
//  world >>FILE("z.ors");
//  world.removeUselessBodies();
//  world >>FILE("z.ors");
//  world.topSort();
//  world.makeLinkTree();
//  world.calc_q_from_Q();
//  world.calc_fwdPropagateFrames();
//  world >>FILE("z.ors");

//  if(MT::checkParameter<bool>("cleanOnly")) return;

  for(;;){
    animateConfiguration(world);
    world.gl().watch();
  }
}

//===========================================================================

int main(int argc,char **argv){

  testReachable();
//  sample();

  return 0;
}
