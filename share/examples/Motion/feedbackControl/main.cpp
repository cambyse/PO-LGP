#include <Ors/ors.h>
#include <Motion/feedbackControl.h>

void reach(){
  ors::KinematicWorld world("man.ors");
  arr q, qdot;
  world.getJointState(q, qdot);

  FeedbackMotionControl MP(world, false);
  MP.addPDTask("endeff1", .2, .8, posTMT, "handR", NoVector, "rightTarget");
  MP.addPDTask("endeff2", .2, .8, posTMT, "handL", NoVector, "leftTarget");

  double tau=0.01;
  for(uint i=0;i<1000;i++){
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

void checkAnalytics(){
  ors::KinematicWorld world("man.ors");
  arr q, qdot;
  world.getJointState(q, qdot);

  FeedbackMotionControl MP(world, false);
  MP.qitselfPD.setGains(1.,10.);
  PDtask *t=MP.addPDTask("endeff1", .2, .9, posTMT, "handR", NoVector, "rightTarget");

  q(18)+=1.1;
  MP.setState(q, qdot);

  ofstream fil("z.approach");
  MT::arrayBrackets="  ";
  double tau=0.01;
  for(uint i=0;i<100;i++){
    MP.setState(q, qdot);

    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
      MP.setState(q, qdot);
      fil <<(i*tau+tt*0.1*tau) <<' ' <<t->y <<length(t->y - t->y_ref) <<endl;
    }
//    fil <<i*tau <<' ' <<q <<endl;

    world.watch(false, STRING(i));
  }
  fil.close();
  gnuplot("plot 'z.approach' us 1:3, '' us 1:(-$4), '' us 1:5, '' us 1:2", false, true);
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

//  reach();
  checkAnalytics();

  return 0;
}
