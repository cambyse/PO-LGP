#include <Core/util.h>
#include <iostream>
#include <Roopi/roopi.h>
#include <Control/taskControl.h>
#include <Motion/komo.h>
#include <RosCom/subscribeRosKinect.h>
#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/percept.h>
#include <Kin/kinViewer.h>

void moveS1S2(Roopi& R) {
Access<mlr::KinematicWorld> modelWorld("modelWorld");
 modelWorld.readAccess();
 mlr::Vector targetpos = modelWorld->getShapeByName("S2")->X.pos+mlr::Vector(0.,0., .04);
 modelWorld.deAccess();

 for (unsigned int i=0; i<100;++i) { //TODO: check convergence
   modelWorld.writeAccess();
   cout << (modelWorld->getShapeByName("S1")->X.pos-targetpos).length() << " error" << endl;

   modelWorld->setAgent(1);
   TaskControlMethods taskController(modelWorld());
   arr q=modelWorld().q;
   mlr::Shape* shape = modelWorld->getShapeByName("S1");
   mlr::Body *b = shape->body;
   CtrlTask *t;
   t = new CtrlTask(STRING("syncPos_" <<b->name), new TaskMap_Default(posTMT, b->shapes.first()->index));
   
   t->ref = new MotionProfile_Const( targetpos.getArr() );
   taskController.tasks.append(t);

   double cost=0.;
   taskController.updateCtrlTasks(0., modelWorld()); //computes their values and Jacobians
   arr dq = taskController.inverseKinematics(NoArr, NoArr, &cost);
   q += dq;
   
   listDelete(taskController.tasks); //cleanup tasks
  
   modelWorld->setAgent(1);
   modelWorld->setJointState(q);
   modelWorld->setAgent(0);
   
   modelWorld.deAccess();
 }
}


int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  Roopi R(true, false);

  R.getTaskController().lockJointGroupControl("base");  
  OrsViewer v1("modelWorld");
  R.wait();
  moveS1S2(R);
  R.wait();
  return 0;
}
