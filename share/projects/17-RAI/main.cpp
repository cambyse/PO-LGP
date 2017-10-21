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

//===============================================================================

/*/
 * TODO
 *
 * time: pass dt intervals; everything in proper seconds; phase in seconds
 *
 * we need a 'model world' as well: the sim gives position of objects -> integrate in modelworld
/*/

//===============================================================================

int planPath(const mlr::String& cmd){
  Access<mlr::KinematicWorld> K("tailKin");
  if(!K.get()->q.N) K.set() = Access<mlr::KinematicWorld>("world").get();
  KOMO_fineManip komo(K.get());

  komo.setPathOpt(1., 30, 5.);

  if(cmd=="grasp") komo.setFineGrasp(1., "endeff", "stick", "wsg_50_base_joint_gripper_left");
  if(cmd=="place") komo.setFinePlace(1., "endeff", "stick", "table1", "wsg_50_base_joint_gripper_left");
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
  Roopi R;
  R.startTweets();

  Access<mlr::KinematicWorld>("world").set()->init("model.g");
  Access<StringA>("jointNames").set() = Access<mlr::KinematicWorld>("world").get()->getJointNames();
  KinSim sim;
  sim.threadLoop();
  Access<double> ttg("timeToGo");

  for(;;){
    planPath("grasp");

    for(;;){ ttg.waitForNextRevision(); if(ttg.get()==0.) break;  }
    Access<arr>("path").set() = Access<arr>("plan").get();
    planPath("home");

    for(;;){ ttg.waitForNextRevision(); if(ttg.get()==0.) break;  }
    Access<StringA>("switches").set() = {"attach", "endeff", "nostick"};
    Access<arr>("path").set() = Access<arr>("plan").get();
    planPath("place");

    for(;;){ ttg.waitForNextRevision(); if(ttg.get()==0.) break;  }
    Access<arr>("path").set() = Access<arr>("plan").get();
    planPath("home");

    for(;;){ ttg.waitForNextRevision(); if(ttg.get()==0.) break;  }
    Access<StringA>("switches").set() = {"attach", "table1", "nostick"};
    Access<arr>("path").set() = Access<arr>("plan").get();
  }


}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testPickAndPlace2();

  return 0;
}

