#include <Motion/komo.h>

#include <observation_tasks.h>

using namespace std;

//===========================================================================

void move(){
  
  KOMO komo;
  komo.setConfigFromFile();

  // mlr::Body *b = komo.world.getBodyByName("/human/base");
  // b->X.addRelativeTranslation(.3,0,0);

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  //komo.setPosition(1., 1.1, "humanL", "target", OT_sumOfSqr, NoArr, 1e2);

  //komo.setPosition(1., 2.0, "manhead", "target", OT_sumOfSqr, ARR(0,0,0), 1e2);  // objective to have the head on the target on this time slab
  //komo.setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);

  //arr targetArr1 = HeadPoseMap::buildTarget( mlr::Vector( 0, -0.3, 1.7 ), 80 );
  //komo.setTask(1.0, 3.0, new HeadPoseMap(), OT_sumOfSqr, targetArr1, 1e2);

  komo.setTask( 1.0, 2.0, new HeadGetSight( ARR(  0.0, -1.0, 1.9 ),    // object position
                                            ARR( -0.2, -0.6, 1.9 ) ),  // pivot position
                                            OT_sumOfSqr, NoArr, 1e2 );

//  komo.setTask( 1.0, 2.0, new HeadGetSight( ARR(  1.0, -0.0, 1.9 ),    // object position
//                                            ARR(  1.0, -0.0, 1.9 ) ),  // pivot position
//                OT_sumOfSqr, NoArr, 1e2 );

  //komo.setTask(.3, .5, new HandPositionMap(), OT_sumOfSqr, ARR(.5,.5,1.3), 1e2);
  //komo.setTask(.8, 1., new HandPositionMap(), OT_sumOfSqr, ARR(.8,0.,1.3), 1e2);
  //komo.setTask(.8, 1., new TaskMap_Default(posDiffTMT, komo.world, "/human/humanR", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

//  komo.setTask(.3, 1., new TaskMap_Default(gazeAtTMT, komo.world, "eyes", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

  komo.setSlowAround( 3.0, .1, 1e3 );

  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  move();

  return 0;
}
