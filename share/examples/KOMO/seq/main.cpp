#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>

#include <Motion/komo.h>

#include <Ors/ors_swift.h>

//===========================================================================

void TEST(Cooperation){
  KOMO komo;
  komo.setConfigFromFile();

  //-- cost terms
  komo.setHoming();
  if(komo.MP->k_order==2){
    komo.setSquaredQAccelerations();
  }else{
    komo.setSquaredQVelocities();
  }

  komo.setGrasp(1., "handR", "Long1" );
  komo.setHoldStill(1., 2., "humanGraspJointR");
  komo.setPlace(2., "handR", "Long1", "table" );

  komo.setHoldStill(1., 2., "baxterGraspJointR");
  komo.setGrasp(1., "baxterR", "Handle" );
  komo.setPlace(2., "baxterR", "Handle", "Long1" );

  komo.setGrasp(2., "handL", "Long2" );
  komo.setHoldStill(2., 3., "humanGraspJointL");
  komo.setPlace(3., "handL", "Long2", "Handle" );

  komo.reset();
  komo.MP->reportFull(true, FILE("z.problem"));
  komo.run();
  if(komo.MP->T<10){
    for(;;) komo.displayTrajectory(-1.);
  }else{
    for(;;) komo.displayTrajectory(.01);
  }
}


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  testCooperation();

  return 0;
}
