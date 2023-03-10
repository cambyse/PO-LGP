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

//===============================================================================

void TEST(Basics) {
  {
    Roopi R(true);
    {
      auto posL = R.newCtrlTask();
      posL->setMap(new TaskMap_Default(posTMT, R.getK(), "endeffL"));
      posL->task->PD().setTarget( posL->y0 + ARR(0,-.1,-.3) );
      posL->task->PD().setGainsAsNatural(1., .9);
      posL->start();

      R.wait(+posL);
    }

    R.kinematicSwitch("obj1", "endeffL", false);

    {
      auto h = R.home();
      R.wait(+h);
    }
  }
  cout <<"LEFT OVER REGISTRY:\n" <<registry() <<endl;
}

//===============================================================================

void TEST(Homing) {
  {
    Roopi R(true);
    {
      auto h = R.home();
      R.wait(+h);
    }
  }
  cout <<"LEFT OVER REGISTRY:\n" <<registry() <<endl;
}

//===============================================================================

void TEST(Gripper) {
  Roopi R(true);
  //  auto lim = R.newLimitAvoidance();
  Script_setGripper(R, LR_left, .08);
  Script_setGripper(R, LR_left, .0);
}

//===============================================================================

void TEST(PhysX) {
  {
    Roopi R(true);

    auto ph = R.PhysX();
    //    ph.showInternalOpengl();

    auto g = R.graspBox("obj2", LR_left);

    R.wait(+g);

    auto p = R.place("obj2", "objTarget");

    R.wait(+p);

  }
}

//===============================================================================

void Prototyping(){
  Roopi R(true);

  auto view = R.CameraView();

  {
    mlr::Frame *s = R.getK()->getFrameByName("endeffL");
    auto leftTarget = R.newMarker("targetL", conv_vec2arr(s->X.pos)+ARR(.0,-.2,.3));

    auto leftHand = R.newCtrlTask();
    leftHand->setMap(new TaskMap_Default(posDiffTMT, R.getK(), "endeffL", NoVector, "targetL"));
    leftHand->task->PD().setGainsAsNatural(1., .8);
    leftHand->task->PD().setTarget( {.0} );
    leftHand->start();

    auto rightHand = R.newCtrlTask();
    rightHand->setMap(new TaskMap_Default(posDiffTMT, R.getK(), "endeffR"));
    rightHand->task->PD().setGainsAsNatural(1., .8);
    rightHand->task->PD().setTarget( rightHand->y0 + ARR(.0, .2, .6) );
    rightHand->start();

    for(;;){
      R.wait(leftHand+rightHand, 3.); //with timeout
      if(leftHand->getStatus()==AS_converged && rightHand->getStatus()==AS_converged) break; //good
      if(leftHand->getStatus()==AS_stalled && leftHand->time()>5.){
        cout <<"leftHand failed - taking back" <<endl;
        leftHand->set()->PD().setTarget( leftHand->y0 );
      }
      if(rightHand->getStatus()==AS_stalled && rightHand->time()>5.){
        cout <<"rightHand failed - taking back" <<endl;
        rightHand->set()->PD().setTarget( rightHand->y0 );
      }
    }

    leftTarget->X.pos.z -=.3;
    //    leftHand->set()->PD().setTarget( leftHand->y0 );
    rightHand->set()->PD().setTarget( rightHand->y0 );
    R.wait(leftHand+rightHand, 3.); //with timeout
  } //scope check's previous kill


#if 0
  auto path = R.newJointPath(jointState, 5.0)
              ->start();

  R.wait({path}, 4.);
  if(path->getStatus()!=AS_done){
    cout <<"not done yet!" <<endl;
  }
#endif

#if 0 //PLAN
  switchToKinestheticTeachingMode();

  recordPose(taskSpace..);
  recordTrajectory();
  newLog(); // type= arr, image, ctrlMsg, jointAndFTSensors, ctrlTasks (including force) //of only ONE variable (type arr)
  newImageLog(); //of only ONE variable (type byteA)
  newControllerLog();

  *every act should ahve its own log (like LOG..)
    #endif
}

//===============================================================================

void TEST(PickAndPlace) {
  Roopi R(true);

  //  auto view = R.CameraView();
  //  auto pcl = R.newKinect2Pcl();
  //  R.taskController().verbose(1);

  R.getTaskController().lockJointGroupControl("torso");
  R.collisions(true);

  //  auto ph = R.newPhysX();
  //  auto rec = Act_Recorder(&R, "ctrl_q_ref", 10);

//  auto pub = R.rosPublish("modelWorld", .1);

#if 0
  Script_graspBox(R, "obj1", LR_right);
  Script_place(R, "obj1", "objTarget");
#else
  {
    auto ws = R.workspaceReady("obj2");
    R.wait(.5);
    R.deactivateCollisions("coll_hand_l", "obj2");
    auto pick1 = R.graspBox("obj2", LR_left);
    R.wait(.5);
    R.deactivateCollisions("coll_hand_r", "obj1");
    auto pick2 = R.graspBox("obj1", LR_right);
    R.wait(pick1+pick2);
  }{
    auto ws = R.workspaceReady("objTarget");
    auto place1 = R.place("obj2", "objTarget");
    R.wait(+place1);
  }{
    auto place2 = R.place("obj1", "obj2");
    R.wait(+place2);
  }
#endif

  auto home = R.home();
  R.wait(+home);
}

//===============================================================================

void Script_focusWorkspace(Roopi& R, const char* objName){
  //attention
  auto look = R.newCtrlTask(new TaskMap_Default(gazeAtTMT, R.getK(), "endeffKinect", NoVector, objName));
  auto ws = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getK(), "endeffWorkspace", NoVector, objName), {}, {}, {1e1});

  R.wait(ws+look);
}

void TEST(PickAndPlace2) {
  Roopi R(true);

  //  auto view = R.newCameraView();
  //  R.taskController().verbose(1);

  Script_focusWorkspace(R, "obj1");
  R.getTaskController().lockJointGroupControl("base");

  {
    auto path = R.newPathOpt();
    double t1=.75;
    arr obj1size = R.getK()->getFrameByName("obj1")->shape->size();
    double gripSize = obj1size(1) + 2.*obj1size(3);
    double above = obj1size(2)*.5 + obj1size(3) - .02;

    mlr::KinematicWorld& K = path->komo->world;
    path->komo->useJointGroups({"armR", "gripR"});
    path->komo->setPathOpt(1, 20, 5.);
    path->komo->setTask(t1, 1., new TaskMap_Default(posDiffTMT, K, "endeffWorkspace", NoVector, "obj1"), OT_sumOfSqr, {}, 1e1);
    path->komo->setTask(t1, 1., new TaskMap_Default(vecTMT, K, "pr2R", Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e0);
    path->komo->setTask(t1, t1, new TaskMap_Default(posDiffTMT, K, "pr2R", NoVector, "obj1", NoVector), OT_sumOfSqr, {0.,0.,above+.1}, 1e3);
    path->komo->setTask(t1, 1., new TaskMap_Default(vecAlignTMT, K, "pr2R", Vector_x, "obj1", Vector_y), OT_sumOfSqr, NoArr, 1e1);
    path->komo->setTask(t1, 1., new TaskMap_Default(vecAlignTMT, K, "pr2R", Vector_x, "obj1", Vector_z), OT_sumOfSqr, NoArr, 1e1);
    //open gripper
    path->komo->setTask(t1, .85, new TaskMap_qItself(QIP_byJointNames, {"r_gripper_joint"}, K), OT_sumOfSqr, {gripSize + .05});
    path->komo->setTask(t1, .85, new TaskMap_qItself(QIP_byJointNames, {"r_gripper_l_finger_joint"}, K), OT_sumOfSqr, {::asin((gripSize + .05)/(2.*.10))});
    //close gripper
    path->komo->setTask(.95, 1., new TaskMap_qItself(QIP_byJointNames, {"r_gripper_joint"}, K), OT_sumOfSqr, {gripSize});
    path->komo->setTask(.95, 1., new TaskMap_qItself(QIP_byJointNames, {"r_gripper_l_finger_joint"}, K), OT_sumOfSqr, {::asin((gripSize)/(2.*.10))});
    path->komo->setTask(.9, 1., new TaskMap_Default(posDiffTMT, K, "pr2R", NoVector, "obj1", NoVector), OT_sumOfSqr, {0.,0.,above}, 1e3);
    path->komo->setSlowAround(1., .05, 1e3);
    path->start();


    R.wait(+path);

    auto follow = Act_FollowPath(&R, "PathFollower", path->komo->x, new TaskMap_qItself(QIP_byJointGroups, {"armR","gripR"}, R.getK()), 5.);
    follow.start();

    R.wait({&follow});
  }

  {
    arr obj1size = R.getK()->getFrameByName("obj1")->shape->size();
    double gripSize = obj1size(1) + 2.*obj1size(3);
    auto gripperR = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"r_gripper_joint"}, R.getK()), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"r_gripper_l_finger_joint"}, R.getK()), {}, {::asin(gripSize/(2.*.10))});

    R.wait(+gripperR);
  }
}

//===============================================================================

void localizeS1(Roopi &R, const char* obj){
  {
    //    auto L = R.lookAt("S3");
    auto look = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"head_tilt_joint"}, R.getK()), {}, {55.*MLR_PI/180.});
    R.wait(+look);



  auto pcl = R.PclPipeline(true);
  auto filter = R.PerceptionFilter(true);

  Access<PerceptL> outputs("percepts_filtered");
  int rev=outputs.getRevision();
  outputs.waitForRevisionGreaterThan(rev+10);


  cout <<"GRASPING " <<obj <<endl;
  look->stop();
  auto L = R.lookAt(obj, 1e-1);
  auto laser = R.lookAt(obj, 1e-1, "endeffLaser");
  R.wait(+L);
  R.wait(+laser);

  R.wait(3.);
  R.wait();
  }
}

//===============================================================================

void TEST(Perception) {
  Roopi R(true, false);

  R.getTaskController().lockJointGroupControl("base");

  const char* obj="S1";
  OrsViewer v1("modelWorld");

#if 1 //on real robot!
  auto subKin = R.rosKinect(); //SubscribeRosKinect subKin; //subscription into depth and rgb images
//  SubscribeRosKinect2PCL subKin; //direct subscription into pcl cloud
#else //in simulation: create a separate viewWorld
  Access<mlr::KinematicWorld> c("viewWorld");
  c.writeAccess();
  c() = R.variable<mlr::KinematicWorld>("modelWorld").get();
  c().getShapeByName("S1")->X.pos.x += .05; //move by 5cm; just to be different to modelWorld
  c().getShapeByName("S1")->X.rot.addZ(.3); //move by 5cm; just to be different to modelWorld
  c.deAccess();
//  OrsViewer v3("viewWorld");
  auto view = R.CameraView(false, "modelWorld"); //generate depth and rgb images from a modelWorld view
#endif

  ImageViewer v2("kinect_rgb");

  for(uint k=0;k<4;k++){
    LeftOrRight lr = LeftOrRight(k%2);
    localizeS1(R, obj);

//      R.getComPR2().stopSendingMotionToRobot(true);

    {//pick
      auto g=R.graspBox(obj, lr);
      R.wait(+g);
    }
    {//place
      auto g=R.place(obj, "objTarget");
      R.wait(+g);
    }
    {
      auto home = R.home();
      R.wait(+home);
    }
  }


  R.reportCycleTimes();
}

//===============================================================================

void TEST(PerceptionOnly) {
  Roopi R(true);

  R.getTaskController().lockJointGroupControl("base");

  OrsViewer v1("modelWorld");

  auto kin = R.rosKinect();  //  SubscribeRosKinect subKin; //subscription into depth and rgb images
  ImageViewer v2("kinect_rgb");

  //    auto L = R.lookAt("S3");
  auto look = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"head_tilt_joint"}, R.getK()), {}, {55.*MLR_PI/180.});
  R.wait(+look);

  Act::Ptr view;
  if(!R.useRos()){ //in simulation: create a separate viewWorld
    Access<mlr::KinematicWorld> c("viewWorld");
    c.writeAccess();
    c() = R.variable<mlr::KinematicWorld>("modelWorld").get();
    c().getFrameByName("S1")->X.pos.x += .05; //move by 5cm; just to be different to modelWorld
    c().getFrameByName("S1")->X.rot.addZ(.3); //move by 5cm; just to be different to modelWorld
    c.deAccess();
//    OrsViewer v2("viewWorld");
    view = R.CameraView("viewWorld"); //generate depth and rgb images from a modelWorld view
  }

  auto pcl = R.PclPipeline(true);
  auto filter = R.PerceptionFilter(true);

  auto outputs = R.variableStatus("percepts_filtered");
  int rev=outputs->getStatus();
  outputs->waitForStatusGreaterThan(rev+10);

  R.wait();

  R.reportCycleTimes();
}

//===============================================================================

void TEST(Gamepad) {
  Roopi R(true);

  //  R.taskController().verbose(1);

  //  R.taskController().lockJointGroupControl("base");

  auto gamepad = R.GamepadControl();

  R.wait(+gamepad, -1.);
}


//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  //--very simple one action tests
//  testHoming(); return 0;
//  testBasics();
//  testGripper();
//  testPhysX();

//  Prototyping();

//  testPerception();
//  testPerceptionOnly();

  for(;;) testPickAndPlace();

  for(;;) testPickAndPlace2();
//  testGamepad();

  return 0;
}

