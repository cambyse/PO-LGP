#include <Core/module.h>
#include <Control/TaskControllerModule.h>
#include <Roopi/roopi.h>
#include <Roopi/loggingModule.h>

#include "poseGenerator.h"

void sampleData() {
  Roopi R;
  SetOfDataFiles logging("gcKugel_deleteMe");
  rnd.clockSeed();
  PoseGenerator poser(R.tcm()->modelWorld.get()());
  
  for(uint i = 0; i < 1000; i++) {
    //try to find both a suitable random pose and a trajectory that transfers from the current configuration to the random pose
    while(true) {
      arr qPose = poser.getRandomPose();
      if(R.gotToJointConfiguration(qPose,5.0, true)) break;
    }
    mlr::wait(0.5);
    R.holdPosition();
    mlr::wait(1.0);
    double limits = R.getLimitConstraint(0.02); //TODO what margin. Limits are arghrgrhrgh
    cout << "Limits: " << limits << endl;
    if(limits > 0) {
      cout << "not good" << endl;
      mlr::wait(5.0);
      continue;
    }
    //wait a few seconds to ensure that the controller has converged
    mlr::wait(3.0);
    
    //mean over 1 second with 10 samples
    arr q, qSign, fL, fR, u;
    for(uint j = 0; j < 10; j++) {
      q.append(~R.getJointState());
      qSign.append(~R.getJointSign());
      fL.append(~R.getFTLeft());
      fR.append(~R.getFTRight());
      u.append(~R.getTorques());
      mlr::wait(0.1);
    }
    //save everything
    logging.write("q", sum(q,0)/(double)q.d0);
    logging.write("qSign", sum(qSign,0)/(double)qSign.d0); //TODO does it make sense to mean over them? NO!!
    logging.write("u", sum(u,0)/(double)u.d0);
    logging.write("fL", sum(fL,0)/(double)fL.d0);
    logging.write("fR", sum(fR,0)/(double)fR.d0);
  }
}

void testCollision() {
  Roopi R;
  makeConvexHulls(R.tcm()->modelWorld.set()->shapes);
  CollisionConstraint c(0.1);
  while(true) {
    R.tcm()->modelWorld.set()->stepSwift();
    arr y;
    c.phi(y, NoArr, R.tcm()->modelWorld.get()());
    cout << y << endl;
    mlr::wait(0.5);
  }
}


void learnModel() {
  GravityCompensation gc(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  
  gc.learnGCModel();
  
  StringA joints;
  joints.append(gc.leftJoints);
  joints.append(gc.rightJoints);
  joints.append(gc.headJoints);
  
  cout << gc.compensate(gc.world.getJointState(), gc.world.getJointState(), joints) << endl;
  //gc.learnFTModel();
  
  //cout << gc.compensateFTL(gc.world.getJointState()) << endl;
}

void testOnRobot() {
  Roopi R;
  CtrlTask* t = R.createCtrlTask("damping", new TaskMap_qItself);
  R.modifyCtrlTaskGains(t, .0,1.0);
  R.modifyCtrlC(t, ARR(1000.0));
  R.activateCtrlTask(t);
  R.releasePosition();
  mlr::wait(1000.0);
}

void testFTCompensation() {
  Roopi R;
  PoseGenerator poseGenerator(R.tcm()->modelWorld.get()());
  while(true) {
    arr pose = poseGenerator.getRandomPose();
    R.gotToJointConfiguration(pose, 10.0, true);
    R.holdPosition();
    for(uint i = 0; i < 10; i++) {
      cout << R.getFTRight() << endl;
      mlr::wait(1.0);
    }
  }
}

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  //sampleData();
  //learnModel();
  //testCollision();
  //testOnRobot();
  testFTCompensation();
  return 0;
}
