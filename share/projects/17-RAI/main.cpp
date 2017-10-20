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

struct KOMO_fineManip : KOMO{
  KOMO_fineManip(const mlr::KinematicWorld& K) : KOMO(K){}

  void setFineGrasp(double time, const char *endeff, const char *object, const char* gripper){
    mlr::KinematicWorld& K = world;
    StringA joints = K.getJointNames();

    setKinematicSwitch(time, true, "JT_transX", endeff, object);
//    setKinematicSwitch(time, true, "insert_transX", NULL, object);

    //vertical
    setTask(time-.2, time, new TaskMap_Default(vecTMT, K, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e1);
    //downward motion
    setTask(time-.3, time-.2, new TaskMap_Default(posTMT, K, endeff), OT_sumOfSqr, {0.,0.,-.3}, 1e0, 1);
    //anti-podal
//    setTask(time-.3, time, new TaskMap_Default(vecAlignTMT, K, endeff, Vector_y, object, Vector_x), OT_sumOfSqr, NoArr, 1e1);
//    setTask(time-.3, time, new TaskMap_Default(vecAlignTMT, K, endeff, Vector_y, object, Vector_z), OT_sumOfSqr, NoArr, 1e1);
    //insideBox
    setTask(time-.1, time, new TaskMap_InsideBox(K, endeff, NoVector, object), OT_ineq, NoArr, 1e2);
    //open gripper
    setTask(time-.2, time-.1, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.04}, 1e1);
    setTask(time, time, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.01}, 1e1);
    //hold still
    joints.removeValue(gripper);
    setTask(time-.1, time, new TaskMap_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);
  }

};

//===============================================================================

void planGrasp(Roopi& R, const char* output=NULL){
  KOMO_fineManip komo(R.getK());

  komo.setPathOpt(1., 60, 5.);
//  komo.setGrasp(1., "endeff", "stick");

  komo.setFineGrasp(1., "endeff", "stick", "wsg_50_base_joint_gripper_left");

  komo.reset();
  komo.run();
  komo.getReport(true);

  for(;;) komo.displayTrajectory(.05, true);
//  R.wait();

  StringA joints = komo.world.getJointNames();
  arr x = komo.getPath(joints);
  if(!output) output="path";
  Access<arr>(output).set() = x;
}

//===============================================================================

void TEST(PickAndPlace2) {
  Roopi R(true);

  planGrasp(R);

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

