#include <Roopi/roopi.h>
#include <Motion/komo.h>
#include <Control/taskControl.h>
#include <RosCom/subscribeRosKinect.h>
#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/syncFiltered.h>
#include <Kin/kinViewer.h>

#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/publishDatabase.h>
#include <RosCom/baxter.h>

#include "lockbox/script_Lockbox.h"

//===============================================================================


void TEST(Lockbox) {
  {
    Roopi R(false);
    R.setKinematics("../../../data/baxter_model/baxter-lockbox.ors");

    auto roscom = R.newComROS();

    auto filter = R.newPerceptionFilter(true);
    R.startTweets();
    R.startTaskController();

    SubscribeAlvar alvar_subscriber;
    Collector data_collector(!mlr::getParameter<bool>("useRos", false));
    PublishDatabase myPublisher;
    SendPositionCommandsToBaxter spcb(R.getK());
    Access_typed<sensor_msgs::JointState> jointState(NULL, "jointState");
    Subscriber<sensor_msgs::JointState> sub("/robot/joint_states", jointState);

    SyncFiltered sync("modelWorld");
    Lockbox lockbox(&R);

    threadOpenModules(true);

    lockbox.home();
    mlr::wait(3.);
    lockbox.align_lockbox();
    lockbox.move_above("lockbox_marker10", ARR(0., 0., 0.4));
    mlr::wait(3.);
    lockbox.align_lockbox();
    lockbox.home();


    for (uint i = 0; i < 5; i++)
    {
      lockbox.move_joint(i, 1);
      lockbox.home();
    }
    lockbox.move_above("lockbox_marker10");

    {
      auto posL = R.newCtrlTask();
      posL.setMap(new TaskMap_Default(posDiffTMT, R.getK(), "endeffL", NoVector, "lockbox_marker10"));
      posL.task->PD().setTarget( {.0} );
      posL.task->PD().setGainsAsNatural(1., .9);
      posL.start();

      R.wait({&posL});
      mlr::wait();
    }
    {
      lockbox.home();
    }
  }
}


//===============================================================================

void TEST(Basics) {
  {
    Roopi R(true);
    {
      auto posL = R.newCtrlTask();
      posL.setMap(new TaskMap_Default(posTMT, R.getK(), "endeffL"));
      posL.task->PD().setTarget( posL.y0 + ARR(0,-.1,-.3) );
      posL.task->PD().setGainsAsNatural(1., .9);
      posL.start();

      R.wait({&posL});
    }
    {
      auto h = R.home();
      R.wait({&h});
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

    auto ph = R.newPhysX();
    //    ph.showInternalOpengl();

    auto g = R.graspBox("obj2", LR_left);

    R.wait({&g});

    auto p = R.place("obj2", "objTarget");

    R.wait({&p});

  }
}

//===============================================================================

void Prototyping(){
  Roopi R(true);

  auto view = R.newCameraView();

  {
    mlr::Shape *s = R.getK()->getShapeByName("endeffL");
    auto leftTarget = R.newMarker("targetL", conv_vec2arr(s->X.pos)+ARR(.0,-.2,.3));

    auto leftHand = R.newCtrlTask();
    leftHand.setMap(new TaskMap_Default(posDiffTMT, R.getK(), "endeffL", NoVector, "targetL"));
    leftHand.task->PD().setGainsAsNatural(1., .8);
    leftHand.task->PD().setTarget( {.0} );
    leftHand.start();

    auto rightHand = R.newCtrlTask();
    rightHand.setMap(new TaskMap_Default(posDiffTMT, R.getK(), "endeffR"));
    rightHand.task->PD().setGainsAsNatural(1., .8);
    rightHand.task->PD().setTarget( rightHand.y0 + ARR(.0, .2, .6) );
    rightHand.start();

    for(;;){
      R.wait({&leftHand, &rightHand}, 3.); //with timeout
      if(leftHand.getStatus()==AS_converged && rightHand.getStatus()==AS_converged) break; //good
      if(leftHand.getStatus()==AS_stalled && leftHand.time()>5.){
        cout <<"leftHand failed - taking back" <<endl;
        leftHand.set()->PD().setTarget( leftHand.y0 );
      }
      if(rightHand.getStatus()==AS_stalled && rightHand.time()>5.){
        cout <<"rightHand failed - taking back" <<endl;
        rightHand.set()->PD().setTarget( rightHand.y0 );
      }
    }

    leftTarget->rel.pos.z -=.3;
    //    leftHand.set()->PD().setTarget( leftHand.y0 );
    rightHand.set()->PD().setTarget( rightHand.y0 );
    R.wait({&leftHand, &rightHand}, 3.); //with timeout
  } //scope check's previous kill


#if 0
  auto path = R.newJointPath(jointState, 5.0)
              .start();

  R.wait({path}, 4.);
  if(path.getStatus()!=AS_done){
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

  auto view = R.newCameraView();
  //  auto pcl = R.newKinect2Pcl();
  //  R.taskController().verbose(1);

  R.getTaskController().lockJointGroupControl("torso");

  //  auto ph = R.newPhysX();
  //  auto rec = Act_Recorder(&R, "ctrl_q_ref", 10);
  R.collisions(true);

#if 0
  Script_graspBox(R, "obj1", LR_right);
  Script_place(R, "obj1", "objTarget");
#else
  auto pick1 = R.graspBox("obj2", LR_left);
  mlr::wait(.5);
  auto pick2 = R.graspBox("obj1", LR_right);
  R.wait({&pick1,&pick2});

  auto place1 = R.place("obj2", "objTarget");
  R.wait({&place1});
  auto place2 = R.place("obj1", "obj2");
  R.wait({&place2});
#endif

  auto home = R.home();
  R.wait({&home});
}

//===============================================================================

void focusWorkspace_pr2(Roopi& R, const char* objName){
  //attention
  auto look = R.newCtrlTask(new TaskMap_Default(gazeAtTMT, R.getK(), "endeffKinect", NoVector, objName));
  auto ws = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getK(), "endeffWorkspace", NoVector, objName), {}, {}, {1e1});

  R.wait({&ws, &look});
}

void TEST(PickAndPlace2) {
  Roopi R(true);

  //  auto view = R.newCameraView();
  //  R.taskController().verbose(1);

  focusWorkspace_pr2(R, "obj1");
  R.getTaskController().lockJointGroupControl("base");

  {
    auto path = R.newPathOpt();
    double t1=.75;
    arr obj1size(R.getK()->getShapeByName("obj1")->size, 4);
    double gripSize = obj1size(1) + 2.*obj1size(3);
    double above = obj1size(2)*.5 + obj1size(3) - .02;

    mlr::KinematicWorld& K = path.komo->world;
    path.komo->useOnlyJointGroup({"armR", "gripR"});
    path.komo->setPathOpt(1, 20, 5.);
    path.komo->setTask(t1, 1., new TaskMap_Default(posDiffTMT, K, "endeffWorkspace", NoVector, "obj1"), OT_sumOfSqr, {}, 1e1);
    path.komo->setTask(t1, 1., new TaskMap_Default(vecTMT, K, "pr2R", Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e0);
    path.komo->setTask(t1, t1, new TaskMap_Default(posDiffTMT, K, "pr2R", NoVector, "obj1", NoVector), OT_sumOfSqr, {0.,0.,above+.1}, 1e3);
    path.komo->setTask(t1, 1., new TaskMap_Default(vecAlignTMT, K, "pr2R", Vector_x, "obj1", Vector_y), OT_sumOfSqr, NoArr, 1e1);
    path.komo->setTask(t1, 1., new TaskMap_Default(vecAlignTMT, K, "pr2R", Vector_x, "obj1", Vector_z), OT_sumOfSqr, NoArr, 1e1);
    //open gripper
    path.komo->setTask(t1, .85, new TaskMap_qItself(QIP_byJointNames, {"r_gripper_joint"}, K), OT_sumOfSqr, {gripSize + .05});
    path.komo->setTask(t1, .85, new TaskMap_qItself(QIP_byJointNames, {"r_gripper_l_finger_joint"}, K), OT_sumOfSqr, {::asin((gripSize + .05)/(2.*.10))});
    //close gripper
    path.komo->setTask(.95, 1., new TaskMap_qItself(QIP_byJointNames, {"r_gripper_joint"}, K), OT_sumOfSqr, {gripSize});
    path.komo->setTask(.95, 1., new TaskMap_qItself(QIP_byJointNames, {"r_gripper_l_finger_joint"}, K), OT_sumOfSqr, {::asin((gripSize)/(2.*.10))});
    path.komo->setTask(.9, 1., new TaskMap_Default(posDiffTMT, K, "pr2R", NoVector, "obj1", NoVector), OT_sumOfSqr, {0.,0.,above}, 1e3);
    path.komo->setSlowAround(1., .05, 1e3);
    path.start();


    R.wait({&path});

    auto follow = Act_FollowPath(&R, "PathFollower", path.komo->x, new TaskMap_qItself(QIP_byJointGroups, {"armR","gripR"}, R.getK()), 5.);
    follow.start();

    R.wait({&follow});
  }

  {
    arr obj1size(R.getK()->getShapeByName("obj1")->size, 4);
    double gripSize = obj1size(1) + 2.*obj1size(3);
    auto gripperR = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"r_gripper_joint"}, R.getK()), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"r_gripper_l_finger_joint"}, R.getK()), {}, {::asin(gripSize/(2.*.10))});

    R.wait({&gripperR});
  }
}

//===============================================================================

void TEST(Perception) {
  Roopi R(true, false);

  R.getTaskController().lockJointGroupControl("base");

  mlr::Shape *topMost = NULL;
  OrsViewer v2("modelWorld");

  {
    //    auto L = R.lookAt("S3");
    auto look = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"head_tilt_joint"}, R.getK()), {}, {55.*MLR_PI/180.});
#if 0
    auto view = R.newCameraView(false);
#else
    SubscribeRosKinect subKin;
//    SubscribeRosKinect2PCL subKin;
    ImageViewer v1("kinect_rgb");
    subKin.kinect_depth.waitForRevisionGreaterThan(10);
#endif

    auto pcl = R.newPclPipeline(false);
    auto filter = R.newPerceptionFilter(true);
//    mlr::wait();
    {
      SyncFiltered sync("modelWorld");
      mlr::wait(4.);
    }

    for(mlr::Shape *s:R.getK()->shapes){
      if(s->type==mlr::ST_box && s->name.startsWith("perc_")){
        if(!topMost || topMost->X.pos.z < s->X.pos.z) topMost=s;
        cout <<"PERCEIVED: " <<*s <<" X=" <<s->X <<endl;
      }
    }
    cout <<"GRASPING " <<topMost->name <<endl;

    look.stop();
    auto L = R.lookAt(topMost->name);
    mlr::wait();
  }

//  R.getComPR2().stopSendingMotionToRobot(true);
//  return;


  {
    auto g=R.graspBox(topMost->name, LR_left);
    R.wait({&g});
  }

//    R.getComPR2().stopSendingMotionToRobot(true);

  {
    auto g=R.place(topMost->name, "objTarget");
    R.wait({&g});
  }

//  Script_setGripper(R, LR_left, .12);

  {
    auto home = R.home();
    R.wait({&home});
  }


  R.reportCycleTimes();
}
//===============================================================================

void TEST(Gamepad) {
  Roopi R(true);

  //  R.taskController().verbose(1);

  //  R.taskController().lockJointGroupControl("base");

  auto gamepad = R.newGamepadControl();

  R.wait({&gamepad}, -1.);
}


//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testLockbox();
  //  testBasics();
  //  testGripper();
  //  testPhysX();
  //  Prototyping();

  testPerception();

  //  for(;;) testPickAndPlace();

  //  /*for(;;)*/ testPickAndPlace2();
  //  testGamepad();

  return 0;
}
