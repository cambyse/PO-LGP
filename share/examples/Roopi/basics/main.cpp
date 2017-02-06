#include <Roopi/roopi.h>
#include <Motion/komo.h>
#include <Control/taskController.h>

//===============================================================================

void Prototyping(){
  Roopi R(true);

  R.newCameraView();

  {
    mlr::Shape *s = R.getKinematics()->getShapeByName("endeffL");
    auto leftTarget = R.newMarker("targetL", conv_vec2arr(s->X.pos)+ARR(.0,-.2,.3));


    auto leftHand = R.newCtrlTask();
    leftHand.setMap(new TaskMap_Default(posDiffTMT, R.getKinematics(), "endeffL", NoVector, "targetL"));
    leftHand.task->setGainsAsNatural(1., .8);
    leftHand.task->y_ref = .0;
    leftHand.start();

    auto rightHand = R.newCtrlTask();
    rightHand.setMap(new TaskMap_Default(posDiffTMT, R.getKinematics(), "endeffR"));
    rightHand.task->setGainsAsNatural(1., .8);
    rightHand.task->y_ref = rightHand.y0 + ARR(.0, .2, .6);
    rightHand.start();

    R.hold(false);

    for(;;){
      R.wait({&leftHand, &rightHand}, 3.); //with timeout
      if(leftHand.getStatus()==AS_converged && rightHand.getStatus()==AS_converged) break; //good
      if(leftHand.getStatus()==AS_stalled && leftHand.time()>5.){
        cout <<"leftHand failed - taking back" <<endl;
        leftHand.set()->y_ref = leftHand.y0;
      }
      if(rightHand.getStatus()==AS_stalled && rightHand.time()>5.){
        cout <<"rightHand failed - taking back" <<endl;
        rightHand.set()->y_ref = rightHand.y0;
      }
    }

    leftTarget->rel.pos.z -=.3;
//    leftHand.set()->y_ref = leftHand.y0;
    rightHand.set()->y_ref = rightHand.y0;
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

#if 0
  switchToKinestheticTeachingMode();
  recordPose(taskSpace..);
  recordTrajectory();
  newLog(); // type= arr, image, ctrlMsg, jointAndFTSensors, ctrlTasks (including force) //of only ONE variable (type arr)
  newImageLog(); //of only ONE variable (type byteA)
  newControllerLog();

  newPhysicsSimulator();
#endif
}

//===============================================================================

void TEST(PickAndPlace) {
  Roopi R(true);

  R.newCameraView();
  R.verboseControl(1);

  {
    //attention
    auto look = R.newCtrlTask("type=gazeAt ref1=endeffKinect ref2=obj1 PD=[1 .9 0 0]");
    auto ws = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getKinematics(), "endeffWorkspace", NoVector, "obj1"), {}, {}, {1e1});

    //gripper positioning
    auto up = R.newCtrlTask(new TaskMap_Default(vecTMT, R.getKinematics(), "pr2R", Vector_z), {1, .9}, {0.,0.,1.});
    auto pos = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getKinematics(), "pr2R", NoVector, "obj1"), {}, {0.,0.,.2});

    //alignment
#if 0
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getKinematics(), "pr2R", Vector_x, "obj1", Vector_y) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getKinematics(), "pr2R", Vector_y, "obj1", Vector_x) );
#else
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getKinematics(), "pr2R", Vector_x, "obj1", Vector_x) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getKinematics(), "pr2R", Vector_y, "obj1", Vector_y) );
#endif

    //open gripper
    arr obj1size(R.getKinematics()->getShapeByName("obj1")->size, 4);
    double gripSize = obj1size(1) + 2.*obj1size(3) + .05;
    auto gripperR = R.newCtrlTask(new TaskMap_qItself(R.getKinematics()->getJointByName("r_gripper_joint")->qIndex, R.getKinematics()->getJointStateDimension()), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself(R.getKinematics()->getJointByName("r_gripper_l_finger_joint")->qIndex, R.getKinematics()->getJointStateDimension()), {}, {::asin(gripSize/(2.*.10))});

    //go
    R.hold(false);

    R.wait({&pos, &gripperR});

    //lowering
    double above = obj1size(2)*.5 + obj1size(3);
    pos.set()->y_ref = ARR(0,0,above);

    R.wait({&pos});

    //close gripper
    gripSize = obj1size(1) + 2.*obj1size(3);
    gripperR.set()->y_ref = gripSize;
    gripper2R.set()->y_ref = ::asin(gripSize/(2.*.10));

    R.wait({&pos,&gripperR});

    R.kinematicSwitch("obj1","pr2R");
    R.hold(true);
  }

  {
    auto lift = R.newCtrlTask(new TaskMap_Default(posTMT, R.getKinematics(), "pr2R"));
    lift.set()->setTargetToCurrent();
    lift.set()->setGains(0, 10.);
    lift.set()->v_ref = ARR(0,0,.2);

    R.hold(false);

    mlr::wait(1.);
  }


  auto home = R.home();

  R.wait({home});

}

//===============================================================================

void TEST(PickAndPlace2) {
  Roopi R(true);

//  R.newCameraView();
//  R.verboseControl(1);

  if(true){


    //attention
    auto look = R.newCtrlTask("type=gazeAt ref1=endeffKinect ref2=obj1 PD=[1 .9 0 0]");
    auto ws = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getKinematics(), "endeffWorkspace", NoVector, "obj1"), {}, {}, {1e1});

    R.hold(false);
    R.wait({&ws, &look});
    R.lockJointGroupControl("base");
    R.hold(true);
  }

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
    path.komo->setTask(t1, .85, new TaskMap_qItself(K.getJointByName("r_gripper_joint")->qIndex, K.getJointStateDimension()), OT_sumOfSqr, {gripSize + .05});
    path.komo->setTask(t1, .85, new TaskMap_qItself(K.getJointByName("r_gripper_l_finger_joint")->qIndex, K.getJointStateDimension()), OT_sumOfSqr, {::asin((gripSize + .05)/(2.*.10))});
    //close gripper
    path.komo->setTask(.95, 1., new TaskMap_qItself(K.getJointByName("r_gripper_joint")->qIndex, K.getJointStateDimension()), OT_sumOfSqr, {gripSize});
    path.komo->setTask(.95, 1., new TaskMap_qItself(K.getJointByName("r_gripper_l_finger_joint")->qIndex, K.getJointStateDimension()), OT_sumOfSqr, {::asin((gripSize)/(2.*.10))});
    path.komo->setTask(.9, 1., new TaskMap_Default(posDiffTMT, K, "pr2R", NoVector, "obj1", NoVector), OT_sumOfSqr, {0.,0.,above}, 1e3);
    path.komo->setSlowAround(1., .05, 1e3);
    path.start();


    /////////////

    R.wait({&path});

    auto follow = Act_FollowPath(&R, "PathFollower", path.komo->x, new TaskMap_qItself(QIP_byJointGroups, {"armR","gripR"}, R.getKinematics()), 5.);
    follow.start();
    R.hold(false);

    R.wait({&follow});
  }

  {
    arr obj1size(R.getKinematics()->getShapeByName("obj1")->size, 4);
    double gripSize = obj1size(1) + 2.*obj1size(3);
    auto gripperR = R.newCtrlTask(new TaskMap_qItself(R.getKinematics()->getJointByName("r_gripper_joint")->qIndex, R.getKinematics()->getJointStateDimension()), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself(R.getKinematics()->getJointByName("r_gripper_l_finger_joint")->qIndex, R.getKinematics()->getJointStateDimension()), {}, {::asin(gripSize/(2.*.10))});

    R.wait({&gripperR});
    R.hold(true);
  }

  mlr::wait();

}

//===============================================================================

void TEST(Basics) {
  Roopi R;

  R.setKinematics("pr2");
  R.startTaskController();
//  R.verboseControl(1);

  auto posL = R.newCtrlTask();
  posL.setMap(new TaskMap_Default(posTMT, R.getKinematics(), "endeffL"));
  posL.task->y_ref = posL.y0 + ARR(0,0,.3);
  posL.task->setGainsAsNatural(1., .9);
  posL.start();

  R.hold(false);

  R.wait({&posL});

  R.hold(true);


  cout <<"DONE" <<endl;
  mlr::wait(1.);
  cout <<"GONE" <<endl;

}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

//  testBasics();

//  testPickAndPlace();

  testPickAndPlace2();

//  Prototyping();
  return 0;
}
