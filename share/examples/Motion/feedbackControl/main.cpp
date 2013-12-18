#include <Ors/ors.h>
#include <Motion/feedbackControl.h>

void reach(){
  ors::KinematicWorld world("man.ors");
  arr q, qdot;
  world.getJointState(q, qdot);

  FeedbackMotionControl MP(world, false);
  PDtask *x = MP.addPDTask("endeff1", .1, .8, posTMT, "handR", NoVector, "rightTarget");
  MP.addPDTask("endeff2", .1, .8, posTMT, "handL", NoVector, "leftTarget");

  double tau=0.01;
  for(uint i=0;i<1000;i++){
    x->v_ref = ARR(1.,1.,1.);
    MP.setState(q, qdot);
    world.stepPhysx(tau);

    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
    }

    world.watch(false, STRING(i));
  }
}


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  reach();
  return 0;
}
