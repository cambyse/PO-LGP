#include <Roopi/roopi.h>
#include <Motion/komo.h>
#include <Control/taskControl.h>

#include <Control/ctrlMsg.h>

//===============================================================================

void TEST(Basics) {
  {
    Roopi R(true);

//    R.setKinematics("pr2");
//    R.startTaskController();
    //  R.taskController().verbose(1);

    {
      auto posL = R.newCtrlTask();
      posL.setMap(new TaskMap_Default(posTMT, R.getKinematics(), "endeffL"));
      posL.task->PD().setTarget( posL.y0 + ARR(0,-.1,-.3) );
      posL.task->PD().setGainsAsNatural(1., .9);
      posL.start();

      R.hold(false);
      R.wait({&posL});
      R.hold(true);
    }
  }
  cout <<"LEFT OVER REGISTRY:\n" <<registry() <<endl;
}

//===============================================================================

void TEST(PhysX) {
  {
    Roopi R(true);

    auto ph = R.newPhysX();

//    mlr::wait();

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
    mlr::Shape *s = R.getKinematics()->getShapeByName("endeffL");
    auto leftTarget = R.newMarker("targetL", conv_vec2arr(s->X.pos)+ARR(.0,-.2,.3));


    auto leftHand = R.newCtrlTask();
    leftHand.setMap(new TaskMap_Default(posDiffTMT, R.getKinematics(), "endeffL", NoVector, "targetL"));
    leftHand.task->PD().setGainsAsNatural(1., .8);
    leftHand.task->PD().setTarget( {.0} );
    leftHand.start();

    auto rightHand = R.newCtrlTask();
    rightHand.setMap(new TaskMap_Default(posDiffTMT, R.getKinematics(), "endeffR"));
    rightHand.task->PD().setGainsAsNatural(1., .8);
    rightHand.task->PD().setTarget( rightHand.y0 + ARR(.0, .2, .6) );
    rightHand.start();

    R.hold(false);

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

    R.hold(true);

    leftHand.stop();
    rightHand.stop();
  } //scope check's previous kill


#if 0
  auto path = R.newJointPath(jointState, 5.0)
               .start();
  R.hold(false);

  R.wait({path}, 4.);
  if(path.getStatus()!=AS_done){
    cout <<"not done yet!" <<endl;
    R.hold(true);
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

//  auto view = R.newCameraView();
//  R.taskController().verbose(1);

  R.taskController().lockJointGroupControl("torso");

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
  R.wait({home});
}

//===============================================================================

void focusWorkspace_pr2(Roopi& R, const char* objName){
  //attention
  auto look = R.newCtrlTask(new TaskMap_Default(gazeAtTMT, R.getKinematics(), "endeffKinect", NoVector, objName));
  auto ws = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getKinematics(), "endeffWorkspace", NoVector, objName), {}, {}, {1e1});

  R.hold(false);
  R.wait({&ws, &look});
}

void TEST(PickAndPlace2) {
  Roopi R(true);

  auto view = R.newCameraView();
//  R.taskController().verbose(1);


  focusWorkspace_pr2(R, "obj1");
  R.taskController().lockJointGroupControl("base");
  R.hold(true);

  {
    auto path = R.newPathOpt();
    double t1=.75;
    arr obj1size(R.getKinematics()->getShapeByName("obj1")->size, 4);
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

    auto follow = Act_FollowPath(&R, "PathFollower", path.komo->x, new TaskMap_qItself(QIP_byJointGroups, {"armR","gripR"}, R.getKinematics()), 5.);
    follow.start();
    R.hold(false);

    R.wait({&follow});
  }

  {
    arr obj1size(R.getKinematics()->getShapeByName("obj1")->size, 4);
    double gripSize = obj1size(1) + 2.*obj1size(3);
    auto gripperR = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"r_gripper_joint"}, R.getKinematics()), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself(QIP_byJointNames, {"r_gripper_l_finger_joint"}, R.getKinematics()), {}, {::asin(gripSize/(2.*.10))});

    R.wait({&gripperR});
    R.hold(true);
  }

//  mlr::wait();

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

  testBasics();
//  testPhysX();
//  Prototyping();

//  for(;;) testPickAndPlace();

//  /*for(;;)*/ testPickAndPlace2();
//  testGamepad();

  return 0;
}
