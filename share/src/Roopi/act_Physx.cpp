#include "act_Physx.h"
#include <Kin/kin_physx.h>
#include <Kin/kinViewer.h>

struct sAct_PhysX : Thread{
  ACCESS(mlr::KinematicWorld, modelWorld)
  ACCESS(mlr::KinematicWorld, physxWorld)
  ACCESS(arr, ctrl_q_ref)
  PhysXInterface *px;
  OrsViewer *view;
  OpenGL gl;

  sAct_PhysX() : Thread("PhysX", .03), px(NULL){
    threadLoop();
  }

  ~sAct_PhysX(){
    threadClose();
  }

  void open(){
    physxWorld.writeAccess();
    physxWorld() = modelWorld.get();
    for(uint i=physxWorld().joints.N;i--;){
      mlr::Joint *j = physxWorld().joints.elem(i);
      if(j->type==mlr::JT_rigid) delete j;
    }
    physxWorld.deAccess();
    px = new PhysXInterface(physxWorld.set());
    px->setArticulatedBodiesKinematic();
    view = new OrsViewer("physxWorld", .1);
    view->threadLoop();
//    gl.add(glStandardScene);
//    gl.add(*px);
//    gl.camera.setDefault();
  }

  void step(){
    physxWorld.writeAccess();
    physxWorld().setJointState(ctrl_q_ref.get());
    px->step();
    physxWorld.deAccess();
//    gl.update("PhysX internal", false, false, true);
  }

  void close(){
    delete view; view=NULL;
    delete px; px=NULL;
  }
};


Act_PhysX::Act_PhysX(Roopi* r) : Act(r), s(new sAct_PhysX()){}

Act_PhysX::~Act_PhysX(){ delete s; }
