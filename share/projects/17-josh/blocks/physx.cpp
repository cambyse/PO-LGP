#include "physx.h"
#include <Gui/opengl.h>
#include <Kin/frame.h>

void runPhysX(mlr::KinematicWorld& K, double seconds){
  K.gl().camera.setPosition(-5.,-1.,2.);
  K.gl().camera.focus(0,0,1.);
  K.gl().camera.upright();

  FrameL mybodies;
  for(mlr::Frame *b:K.frames) if(b->inertia){
    b->inertia->type = mlr::BT_kinematic;
    if(b->name=="red" || b->name=="yellow" || b->name=="blue"){
      b->inertia->type = mlr::BT_dynamic;
      b->parent->joint->type=mlr::JT_free;
      mybodies.append(b);
    }
  }
  K.reset_q();

  K.watch(true, "BEFORE PhysX -- press ENTER");

  for(uint i=0;i<seconds*100;i++){
    K.physx().step(.01);
    K.watch();
    cout <<i <<' ';
    for(mlr::Frame *b:mybodies) cout <<b->name <<':' <<b->X.pos <<' ';
    cout <<endl;
  }
}
