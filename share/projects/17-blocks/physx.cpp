#include "physx.h"
#include <Gui/opengl.h>

void runPhysX(mlr::KinematicWorld& K, double seconds){
  K.gl().camera.setPosition(-5.,-1.,2.);
  K.gl().camera.focus(0,0,1.);
  K.gl().camera.upright();

  BodyL mybodies;
  for(mlr::Body *b:K.bodies){
    b->type = mlr::BT_kinematic;
    if(b->name=="red" || b->name=="yellow" || b->name=="blue"){
      b->type = mlr::BT_dynamic;
      for(mlr::Joint *j:b->inLinks) j->type=mlr::JT_free;
      mybodies.append(b);
    }
  }
  K.qdim.clear();

  K.watch(true, "BEFORE PhysX -- press ENTER");

  for(uint i=0;i<seconds*100;i++){
    K.physx().step(.01);
    K.watch();
    cout <<i <<' ';
    for(mlr::Body *b:mybodies) cout <<b->name <<':' <<b->X.pos <<' ';
    cout <<endl;
  }
}
