#include <Roopi/roopi.h>
#include <KOMO/komo.h>
#include <Control/taskControl.h>
//#include <RosCom/subscribeRosKinect.h>
//#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/percept.h>
#include <Kin/kinViewer.h>
//#include <memory>
//#include <RosCom/roscom.h>
#include <Kin/frame.h>
#include <Kin/taskMap_InsideBox.h>

#include <Core/thread.h>


//===============================================================================

void planGrasp(Roopi& R, const char* output=NULL){
  KOMO komo(R.getK());
  mlr::KinematicWorld& K = komo.world;
  StringA joints = K.getJointNames();

  komo.setPathOpt(1., 60, 5.);
  komo.setGrasp(1., "endeff", "stick");
  double above = .1;

  komo.setTask(.6, -1., new TaskMap_Default(vecTMT, K, "endeff", Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e0);
  komo.setTask(.7, .9, new TaskMap_Default(posTMT, K, "endeff"), OT_sumOfSqr, {0.,0.,-.3}, 1e1, 1);
  komo.setTask(.9, -1., new TaskMap_Default(posTMT, K, "endeff"), OT_sumOfSqr, {0.,0.,0.}, 1e2, 1);
//    komo.setTask(t1, t1, new TaskMap_Default(posDiffTMT, K, "endeff", NoVector, "stick", NoVector), OT_sumOfSqr, {0.,0.,above+.1}, 1e3);
  komo.setTask(.7, -1., new TaskMap_Default(vecAlignTMT, K, "endeff", Vector_x, "stick", Vector_x), OT_sumOfSqr, NoArr, 1e1);
  komo.setTask(.7, -1., new TaskMap_Default(vecAlignTMT, K, "endeff", Vector_x, "stick", Vector_z), OT_sumOfSqr, NoArr, 1e1);
//    komo.setTask(.9, -1., new TaskMap_Default(posDiffTMT, K, "endeff", NoVector, "stick", NoVector), OT_sumOfSqr, {0.,0.,above}, 1e3);
  komo.setTask(.9, -1., new TaskMap_InsideBox(R.getK(), "endeff", NoVector, "stick"), OT_ineq, NoArr, 1e2);

  //open gripper
  komo.setTask(.9, .9, new TaskMap_qItself(QIP_byJointNames, {"wsg_50_base_joint_gripper_left"}, R.getK()), OT_sumOfSqr, {.04});
  komo.setTask(1., 1., new TaskMap_qItself(QIP_byJointNames, {"wsg_50_base_joint_gripper_left"}, R.getK()), OT_sumOfSqr, {.01});

  komo.reset();
  komo.run();
  komo.getReport(true);

  komo.displayTrajectory();
//  R.wait();

  arr x = komo.getPath(joints);
  if(!output) output="path";
  Access<arr>(output).set() = x;
}

//===============================================================================

void TEST(PickAndPlace2) {
  Roopi R(true);

  {
    auto plan = R.run([&R]()->int{
      planGrasp(R);
      return AS_done;
    });
    R.wait(+plan);
  }

  {
    arr x = Access<arr>("path").get();
    auto follow = Act_FollowPath(&R, "PathFollower", x, new TaskMap_qItself(), 5.);
    follow.start();
    R.wait({&follow});
  }

  {
    auto gripperR = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"wsg_50_base_joint_gripper_left"}, R.getK()), {}, {.01});
    R.wait(+gripperR);
  }

  R.kinematicSwitch("stick", "endeff", false);

  {
    auto h = R.home();
    R.wait(+h);
  }

  R.wait();
}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testPickAndPlace2();

  return 0;
}

