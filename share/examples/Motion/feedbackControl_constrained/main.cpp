#include <Kin/kin.h>
#include <Control/taskController.h>
#include <Motion/taskMaps.h>

void reach(){
  mlr::KinematicWorld world("man.ors");
  arr q, qdot;
  world.getJointState(q, qdot);

  TaskController MP(world, true);
  MP.addPDTask("endeff1", .3, .8, posTMT, "handR", NoVector, "rightTarget");
//  CtrlTask *t = MP.addPDTask("endeff1", .1, .8, posTMT, "handL");
//  t->y_ref={0,-.5,1};

//  ConstraintForceTask *ct = MP.addConstraintForceTask("touchTable", new PlaneConstraint(world, "handL", {0,0,-1,.5}));
  ConstraintForceTask *ct = MP.addConstraintForceTask("touchTable", new PairCollisionConstraint(world, "handL", "table"));
  ct->desiredForce=0.;

  double tau=0.01;
  for(uint i=0;i<1000;i++){
    MP.setState(q, qdot);
//    world.stepPhysx(tau);

    if(i==100) ct->desiredForce=1.;
    if(i==600) ct->desiredForce=0;

    if(i>=300 && i<=330) world.getBodyByName("table")->X.pos.y -=.01;
    if(i>=350 && i<=400) world.getBodyByName("table")->X.pos.y +=.01;

    for(uint tt=0;tt<10;tt++){
      MP.updateConstraintControllers();
      arr a = MP.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
    }

    world.watch(false, STRING(i));
  }
}


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  reach();
  return 0;
}
