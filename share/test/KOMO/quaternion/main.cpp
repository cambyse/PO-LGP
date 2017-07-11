#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

//===========================================================================

void TEST(ReachForQuat){
  mlr::KinematicWorld G("model.g");

  KOMO komo;
  komo.setModel(G);
  komo.setPathOpt(2);
//  komo.setSquaredQVelocities(-1., -1., 1.);
  komo.setSquaredQuaternionNorms(-1., -1., 1e3);

//  komo.setGrasp(1., "endeff", "target");
//  komo.setPosition(1., 1., "endeff", "target");
  komo.setTask(1., -1., new TaskMap_Default(posDiffTMT, komo.world, "endeff", NoVector, "target", NoVector));
  komo.setTask(1., -1., new TaskMap_Default(quatDiffTMT, komo.world, "endeff", NoVector, "target", NoVector));

//  komo.setSlowAround(1., .2, 1e3);

  komo.reset();
  komo.run();
  komo.checkGradients();
  komo.getReport(true);
  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  testReachForQuat();

  return 0;
}


