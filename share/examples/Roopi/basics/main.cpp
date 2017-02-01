#include <Roopi/roopi.h>
#include <Roopi/CtrlTaskAct.h>
#include <Control/TaskControllerModule.h>

//===============================================================================

void Prototyping(){
  Roopi R(true);

  R.newCameraView();

  {
    mlr::Shape *s = R.getKinematics()->getShapeByName("endeffL");
    auto leftTarget = R.newMarker("targetL", conv_vec2arr(s->X.pos)+ARR(.0,-.2,.3));


    auto leftHand = R.newCtrlTask();
    leftHand.setMap(new TaskMap_Default(posTMT, R.getKinematics(), "endeffL", NoVector, "targetL"));
    leftHand.set()->setGainsAsNatural(1., .8);
    leftHand.set()->y_ref = .0;
    leftHand.start();

    auto rightHand = R.newCtrlTask();
    rightHand.setMap(new TaskMap_Default(posTMT, R.getKinematics(), "endeffR"));
    rightHand.set()->setGainsAsNatural(1., .8);
    rightHand.set()->y_ref += ARR(.0, .2, .6);
    rightHand.start();

    R.hold(false);

    for(;;){
      R.waitAnd({&leftHand, &rightHand}, 3.); //with timeout
      if(leftHand.status()==AS_converged && rightHand.status()==AS_converged) break; //good
      if(leftHand.status()==AS_stalled && leftHand.time()>5.){
        cout <<"leftHand failed - taking back" <<endl;
        leftHand.set()->y_ref = leftHand.y0();
      }
      if(rightHand.status()==AS_stalled && rightHand.time()>5.){
        cout <<"rightHand failed - taking back" <<endl;
        rightHand.set()->y_ref = rightHand.y0();
      }
    }

    leftTarget->rel.pos.z -=.3;
//    leftHand.set()->y_ref = leftHand.y0();
    rightHand.set()->y_ref = rightHand.y0();
    R.waitAnd({&leftHand, &rightHand}, 3.); //with timeout

    R.hold(true);

    leftHand.stop();
    rightHand.stop();
  } //scope check's previous kill


#if 0
  auto path = R.newJointPath(jointState, 5.0)
               .start();
  R.hold(false);

  R.wait({path}, 4.);
  if(path.status()!=AS_done){
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
  R.tcm()->verbose=true;

  {
    auto look = R.newCtrlTask("type=gazeAt ref1=endeffKinect ref2=obj1 PD=[1 .9 0 0]");
    auto up = R.newCtrlTask(new TaskMap_Default(vecTMT, R.getKinematics(), "pr2R", Vector_z), {1, .9}, {0.,0.,1.});
    auto pos = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getKinematics(), "pr2R", NoVector, "obj1", NoVector));
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getKinematics(), "pr2R", Vector_x, "obj1", Vector_x), {1,.9}, {}, {1e1});
//    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, R.getKinematics(), "pr2R", Vector_y, "obj1", Vector_x), {}, {}, {1e1});

    R.hold(false);

    R.waitAnd({&look, &up});
    mlr::wait();
  }

  R.hold(true);

  mlr::wait(3.);

}

//===============================================================================

void TEST(Basics) {
  Roopi R;

  R.setKinematics("pr2");
  R.startTaskController();

  auto posL   = R.createCtrlTask("pos", new TaskMap_Default(posTMT, R.getKinematics(), "endeffL"), true);
  posL->y_ref += ARR(0,0,.3);
  posL->setGainsAsNatural(1., .8);

  R.releasePosition();

  //  R.waitConv({posL, align});

  while (! posL->isConverged() ) {
    mlr::wait(0.05);
  }

  R.holdPosition();


  cout <<"DONE" <<endl;
  mlr::wait(1.);
  cout <<"GONE" <<endl;

}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

//  testBasics();

  testPickAndPlace();

//  Prototyping();
  return 0;
}
