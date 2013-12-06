#include <Ors/ors.h>
#include <Motion/feedbackControl.h>

void reach(){
  ors::KinematicWorld S("man.ors");
  arr q,qdot;
  S.getJointState(q, qdot);

  FeedbackMotionControl MP(S, false);
  MP.addPDTask("endeff1", .1, .8, posTMT, "handR", NoVector, "rightTarget");
  MP.addPDTask("endeff2", .1, .8, posTMT, "handL", NoVector, "leftTarget");

  double tau=0.01;
  for(uint i=0;i<1000;i++){
    MP.setState(q, qdot);
    S.stepPhysx(tau);

    arr a = MP.operationalSpaceControl();
    q += tau*qdot;
    qdot += tau*a;

    S.watch(false, STRING(i));
  }
}


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  reach();
  return 0;
}
