#include <Ors/ors.h>
#include <Motion/feedbackControl.h>

void reach(){
  ors::KinematicWorld world("scene1.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  q=1e-1;
  world.setJointState(q);

  FeedbackMotionControl MP(world, false);
  MP.addPDTask("endeff", 1., 1, posTMT, "endeff", NoVector, "goalRef");

  double tau=0.01;
  for(uint i=0;i<3000;i++){
    MP.setState(q, qdot);
    world.stepPhysx(tau);

    arr a = MP.operationalSpaceControl();
    q += tau*qdot;
    qdot += tau*a;

    world.watch(false, STRING(i));
  }
}


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  reach();
  return 0;
}
