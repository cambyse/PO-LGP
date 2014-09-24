#include <Ors/ors.h>
#include <Motion/feedbackControl.h>

// ============================================================================
// helper function to execute the motion problem
void run(FeedbackMotionControl& MP, ors::KinematicWorld& world) {
  arr q, qdot;
  world.getJointState(q, qdot);

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

// ============================================================================
void test_reach()
{
  ors::KinematicWorld world("man.ors");
  FeedbackMotionControl MP(world, false);

  MP.addPDTask("endeff1", .2, .8, posTMT, "handR", NoVector, "rightTarget");
  MP.addPDTask("endeff2", .2, .8, posTMT, "handL", NoVector, "leftTarget");

  run(MP, world);
}

void test_quatTMT()
{
  ors::KinematicWorld world("man.ors");
  FeedbackMotionControl MP(world, false);

  auto effOrientationL = MP.addPDTask("orientationL", 1., .8, quatTMT, "handL", {0, 0, 0});
  effOrientationL->y_ref = {1, 0, 0, 0};
  effOrientationL->flipTargetScalarProduct = true;

  auto effOrientationR = MP.addPDTask("orientationR", 1., .8, quatTMT, "handR", {0, 0, 0});
  effOrientationR->y_ref = {0, 1, 0, 0};
  effOrientationR->flipTargetScalarProduct = true;

  run(MP, world);
}

void test_qSingleTMT()
{
  ors::KinematicWorld world("man.ors");
  FeedbackMotionControl MP(world, false);

  int jointID = -world.getJointByBodyNames("waist", "back")->qIndex;
  auto task = MP.addPDTask("rotateArm", .3, .8, new DefaultTaskMap(qSingleTMT, jointID));
  task->setTarget({.8});

  jointID = -world.getJointByBodyNames("lhip", "lup")->qIndex;
  auto task2 = MP.addPDTask("rotateArm", .3, .8, new DefaultTaskMap(qSingleTMT, jointID));
  task2->setTarget({-1.8});

  run(MP, world);
}

// ============================================================================
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

// ============================================================================
int main(int argc,char **argv)
{
  MT::initCmdLine(argc,argv);

  // test_reach();
  // checkAnalytics();
  // test_quatTMT();
  // test_reach();
  test_qSingleTMT();

  return 0;
}
