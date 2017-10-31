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

#include "sim.h"
#include "komo_fine.h"
#include "filter.h"
#include "simDrake.h"
#include <Msg/MotionReference.h>

//===============================================================================

/*/
 * TODO
 *
 * time: pass dt intervals; everything in proper seconds; phase in seconds
 *
 * we need a 'model world' as well: the sim gives position of objects -> integrate in modelworld
/*/

//===============================================================================

Msg_MotionReference planPath(const mlr::String& cmd, const mlr::Transformation& where=NoTransformation, bool fromCurrent=true){
  Access<mlr::KinematicWorld> K("filterWorld");
  Access<mlr::KinematicWorld> Ktail("kinTail");
  if(!fromCurrent){
    StringA joints = K.get()->getJointNames();
    arr q = Ktail.get()->getJointState(joints);
    K.set() -> setJointState(q, joints);
  }
  KOMO_fineManip komo(K.get());

  komo.setPathOpt(1., 20, 3.);

  if(cmd=="grasp") komo.setFineGrasp(1., "endeff", "box0", "wsg_50_base_joint_gripper_left");
  if(cmd=="place"){
    komo.setFineLift(0., "endeff");
    komo.setFinePlace(1., "endeff", "box0", "table1", "wsg_50_base_joint_gripper_left");
  }
  if(cmd=="placeFixed"){
    komo.setFineLift(0., "endeff");
    CHECK(!where.isZero(), "");
    mlr::Transformation rel = where / komo.world["table1"]->X;
    komo.setFinePlaceFixed(1., "endeff", "box0", "table1", rel, "wsg_50_base_joint_gripper_left");
  }
  if(cmd=="home")  komo.setFineHoming(1., "wsg_50_base_joint_gripper_left");

  komo.reset();
  komo.run();
  komo.getReport(true);

  while(komo.displayTrajectory(.05, true));

  Ktail.set() = *komo.configurations.elem(-1);

  StringA joints = Access<StringA>("jointNames").get();
  arr x = komo.getPath(joints);
  Msg_MotionReference ref;
  ref.path = x;
  ref.tau = { komo.tau };
  ref.append = true;
  return ref;
}

//===============================================================================

void TEST(PickAndPlace2) {
//  Roopi R;
//  R.startTweets();

  mlr::KinematicWorld K("model.g");
//  K.getFrameByName("stick")->ats.newNode({"percept"});

  Access<mlr::KinematicWorld>("world").set() = K;
  StringA joints = Access<mlr::KinematicWorld>("world").get()->getJointNames();
//  joints.removeValue("slider1Joint");
  Access<StringA>("jointNames").set() = joints;

  SimDrake sim;
//  KinSim sim;
  sim.threadLoop();
  Access<double> ttg("timeToGo");

//  OrsViewer v1("world");
//  OrsViewer v2("kinTail");

  FilterSimple filter;
  filter.threadLoop();

  //wait for robot pose msg
  Access<arr>("currentQ").waitForRevisionGreaterThan(10);

  Msg_MotionReference ref = planPath("home");
  Access<Msg_MotionReference>("MotionReference").set() = ref;
  mlr::wait(1.);
  for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }

  for(uint l=0;;l++){
    ref = planPath("grasp");
    Access<Msg_MotionReference>("MotionReference").set() = ref;
    mlr::wait(1.);
    for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }
    Access<StringA>("switches").set() = {"attach", "endeff", "box0"};
    mlr::wait(.1);

//    ref = planPath("home");
//    Access<Msg_MotionReference>("MotionReference").set() = ref;
//    for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }


    if(l%2){
      ref = planPath("placeFixed", { {.5, -.5, 1.12}, Quaternion_Id } );
    }else{
      ref = planPath("placeFixed", { {.5, .5, 1.12}, Quaternion_Id } );
    }
    Access<Msg_MotionReference>("MotionReference").set() = ref;
    mlr::wait(1.);
    for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }
    Access<StringA>("switches").set() = {"attach", "table1", "box0"};
    mlr::wait(.1);

//    ref = planPath("home");
//    Access<Msg_MotionReference>("MotionReference").set() = ref;
//    for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }
  }


}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testPickAndPlace2();

  return 0;
}

