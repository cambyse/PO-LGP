#include <Roopi/roopi.h>
#include <Control/taskControl.h>
#include <Motion/komo.h>
#include <RosCom/subscribeRosKinect.h>
#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/percept.h>
#include <Kin/kinViewer.h>

//===============================================================================


void TEST(PerceptionOnly) {
  Roopi R(true, false);

  R.getTaskController().lockJointGroupControl("base");

  OrsViewer v1("modelWorld");

  SubscribeRosKinect subKin; //subscription into depth and rgb images
  ImageViewer v2("kinect_rgb");

  //    auto L = R.lookAt("S3");
  auto look = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"head_tilt_joint"}, R.getK()), {}, {55.*MLR_PI/180.});
  R.wait({-look});

#if 1 //on real robot!
//  SubscribeRosKinect2PCL subKin; //direct subscription into pcl cloud
#else //in simulation: create a separate viewWorld
  Access_typed<mlr::KinematicWorld> c("viewWorld");
  c.writeAccess();
  c() = R.variable<mlr::KinematicWorld>("modelWorld").get();
  c().getShapeByName("S1")->X.pos.x += .05; //move by 5cm; just to be different to modelWorld
  c().getShapeByName("S1")->X.rot.addZ(.3); //move by 5cm; just to be different to modelWorld
  c.deAccess();
  OrsViewer v2("viewWorld");
  auto view = R.CameraView(true, "viewWorld"); //generate depth and rgb images from a modelWorld view
#endif

  auto pcl = R.PclPipeline(false);
  auto filter = R.PerceptionFilter(true);

  Access<PerceptL> outputs("percepts_filtered");
  int rev=outputs.getRevision();
  outputs.waitForRevisionGreaterThan(rev+10);


  mlr::wait();


  R.reportCycleTimes();
}

void TEST(PerceptionAndPlace) {
  Roopi R(true, false);

  R.getTaskController().lockJointGroupControl("base");

  OrsViewer v1("modelWorld");

  SubscribeRosKinect subKin; //subscription into depth and rgb images
  ImageViewer v2("kinect_rgb");

  //    auto L = R.lookAt("S3");
  auto look = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"head_tilt_joint"}, R.getK()), {}, {55.*MLR_PI/180.});
  R.wait({-look});

#if 1 //on real robot!
//  SubscribeRosKinect2PCL subKin; //direct subscription into pcl cloud
#else //in simulation: create a separate viewWorld
  Access_typed<mlr::KinematicWorld> c("viewWorld");
  c.writeAccess();
  c() = R.variable<mlr::KinematicWorld>("modelWorld").get();
  c().getShapeByName("S1")->X.pos.x += .05; //move by 5cm; just to be different to modelWorld
  c().getShapeByName("S1")->X.rot.addZ(.3); //move by 5cm; just to be different to modelWorld
  c.deAccess();
  OrsViewer v2("viewWorld");
  auto view = R.CameraView(true, "viewWorld"); //generate depth and rgb images from a modelWorld view
#endif


  auto pcl = R.PclPipeline(false);
  auto filter = R.PerceptionFilter(true);

  {
  auto graspR = R.graspBox("S2", LR_right);
  R.wait({-graspR});
  }
  {
  auto placeR = R.placeDistDir("S2","S1", 0,0,0);
  R.wait({-placeR});
  }
  {
  auto graspR = R.graspBox("S3", LR_right);
  R.wait({-graspR});
  }
  {
  auto placeR = R.placeDistDir("S3","S2", 0,0,0);
  R.wait({-placeR});
  }
  {
  auto graspR = R.graspBox("S4", LR_right);
  R.wait({-graspR});
  }
  {
  auto placeR = R.placeDistDir("S4","S3", 0,0,0);
  R.wait({-placeR});
  }

  Access<PerceptL> outputs("percepts_filtered");
  int rev=outputs.getRevision();
  outputs.waitForRevisionGreaterThan(rev+10);


  mlr::wait();


  R.reportCycleTimes();


}


void workspaceAndArms(Roopi& R, const char* objName){
  auto an = R.armsNeutral();
  auto ws = R.workspaceReady(objName);
  R.wait({-ws,-an});
}

void buildTower(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* objName4, const char* ontoName){
  //obj1, obj2, obj3, and obj4 in a tower
  R.deactivateCollisions("coll_hand_r", objName1);
  R.deactivateCollisions("coll_hand_l", objName2);
  R.deactivateCollisions("coll_hand_r", objName3);
  R.deactivateCollisions("coll_hand_l", objName4);
  R.deactivateCollisions("coll_hand_l", objName1);
  R.deactivateCollisions("coll_hand_r", objName2);
  R.deactivateCollisions("coll_hand_l", objName3);
  R.deactivateCollisions("coll_hand_r", objName4);

  R.deactivateCollisions("table2", objName1);

  {
    workspaceAndArms(R,objName1);
  }
  {
    auto graspR = R.graspBox(objName1, LR_right);
    R.wait({-graspR});
    workspaceAndArms(R,objName2);
  }
  {
    auto graspL = R.graspBox(objName2, LR_left);
    R.wait({-graspL});
    workspaceAndArms(R,ontoName);
  }
  {
    auto placeR = R.placeDistDir(objName1,ontoName, 0,0,0);
    R.wait({-placeR});
    workspaceAndArms(R,objName1);
  }
  {
    auto placeL = R.placeDistDir(objName2,objName1, 0,0,0);
    R.wait({-placeL});
    workspaceAndArms(R,objName3);
  }
  {
    auto graspR2 = R.graspBox(objName3, LR_right);
    R.wait({-graspR2});
    workspaceAndArms(R,objName4);
  }
  {
    auto graspL2 = R.graspBox(objName4, LR_left);
    R.wait({-graspL2});
    workspaceAndArms(R,objName2);
  }
  {
    auto placeR2 = R.placeDistDir(objName3,objName2, 0,0,0);
    R.wait({-placeR2});
    workspaceAndArms(R,objName3);
  }
  {
    auto placeL2 = R.placeDistDir(objName4,objName3, 0,0,0);
    R.wait({-placeL2});
    workspaceAndArms(R,objName4);
  }
  mlr::wait();
}

void buildLine(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* objName4, const char* ontoName1, const char* ontoName2, const char* ontoName3, const char* ontoName4){
  //obj1, obj2, obj3, and obj4 in a line
  R.deactivateCollisions("coll_hand_r", objName1);
  R.deactivateCollisions("coll_hand_l", objName2);
  R.deactivateCollisions("coll_hand_r", objName3);
  R.deactivateCollisions("coll_hand_l", objName4);
  R.deactivateCollisions("coll_hand_l", objName1);
  R.deactivateCollisions("coll_hand_r", objName2);
  R.deactivateCollisions("coll_hand_l", objName3);
  R.deactivateCollisions("coll_hand_r", objName4);

  R.deactivateCollisions("table2", objName1);
  R.deactivateCollisions("table2", objName2);
  R.deactivateCollisions("table2", objName3);
  R.deactivateCollisions("table2", objName4);

  {
    workspaceAndArms(R,objName1);
  }
  {
    auto graspR = R.graspBox(objName1, LR_right);
    R.wait({-graspR});
    workspaceAndArms(R,objName2);
  }
  {
    auto graspL = R.graspBox(objName2, LR_left);
    R.wait({-graspL});
    workspaceAndArms(R,ontoName1);
  }
  {
    auto placeR = R.placeDistDir(objName1,ontoName1,0,0,0);
    R.wait({-placeR});
    workspaceAndArms(R,ontoName2);
  }
  {
    auto placeL = R.placeDistDir(objName2,ontoName2,0,0,0);
    R.wait({-placeL});
    workspaceAndArms(R,objName3);
  }
  {
    auto graspR2 = R.graspBox(objName3, LR_right);
    R.wait({-graspR2});
    workspaceAndArms(R,objName4);
  }
  {
    auto graspL2 = R.graspBox(objName4, LR_left);
    R.wait({-graspL2});
    workspaceAndArms(R,ontoName3);
  }
  {
    auto placeR2 = R.placeDistDir(objName3,ontoName3,0,0,0);
    R.wait({-placeR2});
    workspaceAndArms(R,ontoName4);
  }
  {
    auto placeL2 = R.placeDistDir(objName4,ontoName4,0,0,0);
    R.wait({-placeL2});
    workspaceAndArms(R,objName4);
  }
  mlr::wait();
}

void buildBridge(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* ontoName1, const char* ontoName2){
  //obj1, obj2, and obj3 in a bridge
  R.deactivateCollisions("coll_hand_r", objName1);
  R.deactivateCollisions("coll_hand_l", objName2);
  R.deactivateCollisions("coll_hand_l", objName3);
  R.deactivateCollisions("coll_hand_l", objName1);
  R.deactivateCollisions("coll_hand_r", objName2);
  R.deactivateCollisions("coll_hand_r", objName3);

  R.deactivateCollisions("table2", objName1);
  R.deactivateCollisions("table2", objName2);

  {
    workspaceAndArms(R,objName1);
  }
  {
    auto graspR = R.graspBox(objName1, LR_right);
    R.wait({-graspR});
    workspaceAndArms(R,objName2);
  }
  {
    auto graspL = R.graspBox(objName2, LR_left);
    R.wait({-graspL});
    workspaceAndArms(R,ontoName1);
  }
  {
    auto placeR = R.placeDistDir(objName1,ontoName1,0,0,0);
    R.wait({-placeR});
    workspaceAndArms(R,ontoName2);
  }
  {
    auto placeL = R.placeDistDir(objName2,ontoName2,0,0,0);
    R.wait({-placeL});
    workspaceAndArms(R,objName3);
  }
  {
    auto graspL3 = R.graspBox(objName3, LR_right);
    R.wait({-graspL3});
    workspaceAndArms(R,ontoName1);
  }
  {
    auto placeL3 = R.placeDistDir(objName3,objName1,0.1,0,1);
    R.wait({-placeL3});
    workspaceAndArms(R,objName3);
  }
  mlr::wait();
}

void buildHouse(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* objName4, const char* ontoName1){
  //obj1, obj2, obj3, and obj4 in a house
  R.deactivateCollisions("coll_hand_r", objName1);
  R.deactivateCollisions("coll_hand_r", objName2);
  R.deactivateCollisions("coll_hand_l", objName3);
  R.deactivateCollisions("coll_hand_l", objName4);
  R.deactivateCollisions("coll_hand_l", objName1);
  R.deactivateCollisions("coll_hand_l", objName2);
  R.deactivateCollisions("coll_hand_r", objName3);
  R.deactivateCollisions("coll_hand_r", objName4);

  R.deactivateCollisions("table2", objName1);

  {
    workspaceAndArms(R,objName1);
  }
  {
    auto graspR = R.graspBox(objName1, LR_right);
    R.wait({-graspR});
    workspaceAndArms(R,ontoName1);
  }
  {
    auto placeR = R.placeDistDir(objName1,ontoName1,0,0,1);
    R.wait({-placeR});
    workspaceAndArms(R,objName2);
  }
  {
    auto graspR1 = R.graspBox(objName2, LR_right);
    R.wait({-graspR1});
    workspaceAndArms(R,objName3);
  }
  {
    auto graspL1 = R.graspBox(objName3, LR_left);
    R.wait({-graspL1});
    workspaceAndArms(R,objName1);
  }
  {
    auto placeR1 = R.placeDistDir(objName2,objName1,0.1,0,0);
    R.wait({-placeR1});
    workspaceAndArms(R,objName1);
  }
  {
    auto placeL1 = R.placeDistDir(objName3,objName1,-0.1,0,0);
    R.wait({-placeL1});
    workspaceAndArms(R,objName4);
  }
  {
    auto graspL3 = R.graspBox(objName4, LR_left);
    R.wait({-graspL3});
    workspaceAndArms(R,objName2);
  }
  {
    auto placeL3 = R.placeDistDir(objName4,objName2,-0.1,0,0);
    R.wait({-placeL3});
    workspaceAndArms(R,objName4);
  }
  mlr::wait();
}

void buildBox(Roopi& R, const char* objName1, const char* objName2, const char* objName3, const char* objName4,
              const char* objName5, const char* objName6, const char* objName7, const char* objName8,
              const char* objName9, const char* objName10, const char* ontoName1, const char* ontoName2, const char* ontoName3){
  //obj1 to obj14 in a box (1 to 8 are rectangles, 9 and 10 are cubes
  R.deactivateCollisions("coll_hand_r", objName1);
  R.deactivateCollisions("coll_hand_l", objName2);
  R.deactivateCollisions("coll_hand_r", objName3);
  R.deactivateCollisions("coll_hand_l", objName4);
  R.deactivateCollisions("coll_hand_r", objName5);
  R.deactivateCollisions("coll_hand_l", objName6);
  R.deactivateCollisions("coll_hand_r", objName7);
  R.deactivateCollisions("coll_hand_l", objName8);
  R.deactivateCollisions("coll_hand_r", objName9);
  R.deactivateCollisions("coll_hand_l", objName10);
  R.deactivateCollisions("coll_hand_l", objName1);
  R.deactivateCollisions("coll_hand_r", objName2);
  R.deactivateCollisions("coll_hand_l", objName3);
  R.deactivateCollisions("coll_hand_r", objName4);
  R.deactivateCollisions("coll_hand_l", objName5);
  R.deactivateCollisions("coll_hand_r", objName6);
  R.deactivateCollisions("coll_hand_l", objName7);
  R.deactivateCollisions("coll_hand_r", objName8);
  R.deactivateCollisions("coll_hand_l", objName9);
  R.deactivateCollisions("coll_hand_r", objName10);

  R.deactivateCollisions("table2", objName1);
  R.deactivateCollisions("table2", objName2);
  R.deactivateCollisions("table2", objName3);

  {
    workspaceAndArms(R,objName1);
  }
  {
    auto graspR = R.graspBox(objName1, LR_right);
    R.wait({-graspR});
    workspaceAndArms(R,objName2);
  }
  {
    auto graspL = R.graspBox(objName2, LR_left);
    R.wait({-graspL});
    workspaceAndArms(R,ontoName1);
  }
  {
    auto placeR = R.placeDistDir(objName1,ontoName1,0,0,1);
    R.wait({-placeR});
    workspaceAndArms(R,ontoName3);
  }
  {
    auto placeL = R.placeDistDir(objName2,ontoName3,0,0,1);
    R.wait({-placeL});
    workspaceAndArms(R,objName3);
  }
  {
    auto graspR1 = R.graspBox(objName3, LR_right);
    R.wait({-graspR1});
    workspaceAndArms(R,objName4);
  }
  {
    auto graspL1 = R.graspBox(objName4, LR_left);
    R.wait({-graspL1});
    workspaceAndArms(R,ontoName2);
  }
  {
    auto placeR1 = R.placeDistDir(objName3,ontoName2,0,0,1);
    R.wait({-placeR1});
    workspaceAndArms(R,objName1);
  }
  {
    auto placeL1 = R.placeDistDir(objName4,objName1,0,0,0);
    R.wait({-placeL1});
    workspaceAndArms(R,objName9);
  }
  {
    auto graspR2 = R.graspBox(objName9, LR_right);
    R.wait({-graspR2});
    workspaceAndArms(R,objName10);
  }
  {
    auto graspL2 = R.graspBox(objName10, LR_left);
    R.wait({-graspL2});
    workspaceAndArms(R,objName3);
  }
  {
    auto placeR2 = R.placeDistDir(objName9,objName3,0.1,0,1);
    R.wait({-placeR2});
    workspaceAndArms(R,objName3);
  }
  {
    auto placeL2 = R.placeDistDir(objName10,objName3,-0.1,0,1);
    R.wait({-placeL2});
    workspaceAndArms(R,objName5);
  }
  {
    auto graspR3 = R.graspBox(objName5, LR_right);
    R.wait({-graspR3});
    workspaceAndArms(R,objName6);
  }
  {
    auto graspL3 = R.graspBox(objName6, LR_left);
    R.wait({-graspL3});
    workspaceAndArms(R,objName2);
  }
  {
    auto placeR3 = R.placeDistDir(objName5,objName2,0,0,0);
    R.wait({-placeR3});
    workspaceAndArms(R,objName6);
  }
  {
    auto placeL3 = R.placeDistDir(objName6,objName4,0,0,0);
    R.wait({-placeL3});
    workspaceAndArms(R,objName7);
  }
  {
    auto graspR4 = R.graspBox(objName7, LR_right);
    R.wait({-graspR4});
    workspaceAndArms(R,objName8);
  }
  {
    auto graspL4 = R.graspBox(objName8, LR_left);
    R.wait({-graspL4});
    workspaceAndArms(R,objName5);
  }
  {
    auto placeR4 = R.placeDistDir(objName7,objName5,0,0,0);
    R.wait({-placeR4});
    workspaceAndArms(R,objName9);
  }
  {
    auto placeL4 = R.placeDistDir(objName8,objName9,-0.1,0,1);
    R.wait({-placeL4});
    workspaceAndArms(R,objName8);
  }
  //mlr::wait();
}

void testHRI() {
  Roopi R(true);

//  R.getTaskController().verbose(1);
  //R.getTaskController().lockJointGroupControl("torso");
  //R.hold(false);
  R.collisions(true);
  //mlr::wait();

  //buildTower(R,"cube1","cube2","cube3","cube4","objTarget");
  //buildLine(R,"cube1","cube2","cube3","cube4","objTarget","objTarget5","objTarget6","objTarget7");
  //buildBridge(R,"cube1","cube2","rect1","objTarget","objTarget3");
  //buildHouse(R,"rect1","cube1","cube2","rect2","objTarget");
  buildBox(R,"rect1","rect2","rect3","rect4","rect5","rect6","rect7","rect8","cube1","cube2","objTarget","objTarget5","objTarget6");

  /*
  R.deactivateCollisions("coll_hand_r", "rect1");
  R.deactivateCollisions("coll_hand_l", "rect2");
  R.deactivateCollisions("table2", "rect1");
  R.deactivateCollisions("table2", "rect2");
  {
    workspaceAndArms(R,"rect1");
  }
  {
    auto graspR = R.graspBox("rect1", LR_right);
    R.wait({-graspR});
  }
  {
    workspaceAndArms(R,"rect2");
  }
  {
    auto graspL = R.graspBox("rect2", LR_left);
    R.wait({-graspL});
  }
  {
    workspaceAndArms(R,"objTarget");
  }
  {
    auto placeR = R.placeDistDir("rect1","objTarget",0,0,3);
    R.wait({-placeR});
  }
  {
    workspaceAndArms(R,"objTarget7");
  }
  {
    auto placeL = R.placeDistDir("rect2","objTarget7",0,0,3);
    R.wait({-placeL});
  }
  mlr::wait();
  */

  //Script_graspBox(R,"rect1", LR_right);
  //Script_place(R,"rect1","objTarget");
  //Script_placeDistDir(R,"rect1","objTarget",0.2,0,1);
  //mlr::wait();
  //Script_graspBox(R,"obj1", LR_right);
  //  auto grasp = R.graspBox("obj1", LR_right);
  //  mlr::wait(1.);
  //  auto grasp2 = R.graspBox("obj2", LR_left);
  //graspR = R.place("obj1",LR_right);
  //graspL = R.place("obj2", LR_left);
  //  auto grasp = R.graspBox("obj1", LR_right);
  //  mlr::wait(1.);
  //  auto grasp2 = R.graspBox("obj2", LR_left);
  //R.wait({-graspR, -graspL});

  //mlr::wait();
  //Script_graspBox(R,"obj2", LR_left);

  //mlr::wait();

}


//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  for(;;) testHRI();
//  testPerceptionOnly();
//  testPerceptionAndPlace();

  return 0;
}
