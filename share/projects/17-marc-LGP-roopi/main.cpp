#include <Roopi/roopi.h>

//===============================================================================

void TEST(PickAndPlace) {
  Roopi R(true);

  auto view = R.CameraView();
  //  auto pcl = R.newKinect2Pcl();
  //  R.taskController().verbose(1);

  R.getTaskController().lockJointGroupControl("torso");

  //  auto ph = R.newPhysX();
  //  auto rec = Act_Recorder(&R, "ctrl_q_ref", 10);
  R.collisions(true);

#if 1
  Script_graspBox(R, "obj1", LR_right);
  Script_place(R, "obj1", "objTarget");
#else
  R.deactivateCollisions("coll_hand_l", "obj2");
  auto pick1 = R.graspBox("obj2", LR_left);
  mlr::wait(.5);
  R.deactivateCollisions("coll_hand_r", "obj1");
  auto pick2 = R.graspBox("obj1", LR_right);
  R.wait({-pick1,-pick2});

  auto place1 = R.place("obj2", "objTarget");
  R.wait({-place1});
  auto place2 = R.place("obj1", "obj2");
  R.wait({-place2});
#endif

  auto home = R.home();
  R.wait({-home});
}

//===============================================================================


void TEST(PickAndPlace2) {
  Roopi R(true);

  //  auto view = R.newCameraView();
  //  R.taskController().verbose(1);

  {
  auto look = R.lookAt("obj1");
  {
    auto ws = R.focusWorkspaceAt("obj1");
    R.wait(ws+look);

    R.getTaskController().lockJointGroupControl("base");
  }

  Script_komoGraspBox(R, "obj1", LR_left);

  {
    double gripSize = getGripSize(R, "obj1");
    auto closeGrip = R.setGripper(LR_left, gripSize-.05);
    R.wait(+closeGrip);
  }
  
  //switch
  look.reset();
  }

  R.kinematicSwitch("obj1", "pr2L", false);

  {
    auto lift = R.moveVel("pr2L", {0,0,.2});
    mlr::wait(1.);
  }

  mlr::wait();

}

//============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

//  for(;;) testPickAndPlace();

  for(;;) testPickAndPlace2();

  return 0;
}

