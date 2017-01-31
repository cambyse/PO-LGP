#include <Roopi/roopi.h>
#include <Roopi/CtrlTaskAct.h>

//===============================================================================

void Prototyping(){
  Roopi R(true);

  {
    auto leftHand = R.newCtrlTask();
    leftHand.setMap(new TaskMap_Default(posTMT, R.getKinematics(), "endeffL"));
    leftHand.set()->setGainsAsNatural(1., .8);
    leftHand.set()->y_ref += ARR(.0, .0, .3);
    leftHand.start();

    auto rightHand = R.newCtrlTask();
    rightHand.setMap(new TaskMap_Default(posTMT, R.getKinematics(), "endeffR"));
    rightHand.set()->setGainsAsNatural(1., .8);
    rightHand.set()->y_ref += ARR(.0, .0, .6);
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

    leftHand.set()->y_ref = leftHand.y0();
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

  Prototyping();
  return 0;
}
