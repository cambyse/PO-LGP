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

//===============================================================================

/*/
 * TODO
 *
 * time: pass dt intervals; everything in proper seconds; phase in seconds
 *
 * we need a 'model world' as well: the sim gives position of objects -> integrate in modelworld
/*/

//===============================================================================

int planPath(const mlr::String& cmd, bool fromCurrent=false){
  Access<mlr::KinematicWorld> K("tailKin");
  if(!K.get()->q.N) K.set() = Access<mlr::KinematicWorld>("world").get();
  KOMO_fineManip komo(K.get());
  if(fromCurrent){
    StringA joints = Access<StringA>("jointNames").get();
    arr q = Access<arr>("currentQ").get();
    komo.world.setJointState(q, joints);
  }


  komo.setPathOpt(1., 20, 5.);

  if(cmd=="grasp") komo.setFineGrasp(1., "endeff", "box0", "wsg_50_base_joint_gripper_left");
  if(cmd=="place") komo.setFinePlace(1., "endeff", "box0", "table1", "wsg_50_base_joint_gripper_left");
  if(cmd=="home")  komo.setHoming(.9, 1., 1e2);

  komo.reset();
  komo.run();
//  komo.getReport(true);

//  for(;;)
//  komo.displayTrajectory(.05, true);
//  R.wait();

  StringA joints = Access<StringA>("jointNames").get();
  arr x = komo.getPath(joints);
  Access<arr>("plan").set() = x;
  K.set() = *komo.configurations.elem(-1);
  return 1;
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

//  SimDrake sim;
  KinSim sim;
  sim.threadLoop();
  Access<double> ttg("timeToGo");

//  OrsViewer v1("world");
//  OrsViewer v2("kinTail");

  FilterSimple filter;
  filter.threadLoop();

  //wait for robot pose msg
  Access<arr>("currentQ").waitForRevisionGreaterThan(10);

  planPath("home", true);
  Access<arr>("refPath").set() = Access<arr>("plan").get();

  for(;;){
    planPath("grasp");

    for(;;){ ttg.waitForNextRevision(); if(ttg.get()==0.) break;  }
    Access<arr>("refPath").set() = Access<arr>("plan").get();
    planPath("home");

    for(;;){ ttg.waitForNextRevision(); if(ttg.get()==0.) break;  }
    Access<StringA>("switches").set() = {"attach", "endeff", "box0"};
    Access<arr>("refPath").set() = Access<arr>("plan").get();
    planPath("place");

    for(;;){ ttg.waitForNextRevision(); if(ttg.get()==0.) break;  }
    Access<arr>("refPath").set() = Access<arr>("plan").get();
    planPath("home");

    for(;;){ ttg.waitForNextRevision(); if(ttg.get()==0.) break;  }
    Access<StringA>("switches").set() = {"attach", "table1", "box0ick"};
    Access<arr>("refPath").set() = Access<arr>("plan").get();
  }


}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testPickAndPlace2();

  return 0;
}

