#include <Core/thread.h>
#include <Roopi/roopi.h>
#include <Control/TaskControllerModule.h>

void basicRoopi() {
#if 1
  //this works
  Roopi R;
#else
  //this does work only with debugger
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  Roopi R(world);
#endif

  //Goto specific joint position
  arr jointState = FILE("jointStatePredefined");
  if(!R.gotToJointConfiguration(jointState, 5.0)) return;
  R.holdPosition();

  //EndeffR position task
  CtrlTask* posEndeffR = R.createCtrlTask("posEndeffR", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlC(posEndeffR, ARR(1000.0));
  R.modifyCtrlTaskGains(posEndeffR, 10.0, 5.0);

  //EndeffR orientation task
  CtrlTask* oriEndeffR = R.createCtrlTask("oriEndeffR", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(1.0,0.0,0.0)));
  R.modifyCtrlC(oriEndeffR, ARR(1000.0));
  R.modifyCtrlTaskGains(oriEndeffR, 10.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeffR, ARR(0.0,0.0,-1.0));

  //get actual position and modify it
  arr newPos = R.getTaskValue(posEndeffR);
  newPos(1) += 0.2;


  //activate tasks
  R.activateCtrlTask(posEndeffR);
  R.activateCtrlTask(oriEndeffR);
  R.releasePosition(); //without release, the hold position task will rule!

  //interpolate to new target -- non threaded
  R.interpolateToReference(posEndeffR, 5.0, newPos);

  newPos(0) -= 0.2;
  R.interpolateToReference(posEndeffR, 5.0, newPos);

  newPos(1) -= 0.2;
  R.interpolateToReference(posEndeffR, 5.0, newPos);


  //interpolate to new target -- now with thread!
  TaskReferenceInterpolAct* refT = R.createTaskReferenceInterpolAct("moveEndeffR", posEndeffR);

  newPos(0) += 0.2;
  R.interpolateToReference(refT, 5.0, newPos);
  R.waitForFinishedTaskReferenceInterpolAct(refT);

  newPos(1) -= 0.2;
  R.interpolateToReference(refT, 5.0, newPos);
  R.waitForFinishedTaskReferenceInterpolAct(refT);

  mlr::wait(5.0);
}


int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  basicRoopi();
  return 0;
}
