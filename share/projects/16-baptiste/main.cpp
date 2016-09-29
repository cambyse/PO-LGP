#include <Motion/komo.h>

//===========================================================================

void moveReba(){
  KOMO komo;
  komo.setConfigFromFile();

//  komo.setHoming(-1., -1., 1e-1);
//  komo.setSquaredQVelocities();
  komo.setSquaredQAccelerations();
  komo.setPosition(1., 1., "endeffL", "target", sumOfSqrTT, NoArr, 1e2);
  komo.setSlowAround(1., .1, 1e3);

  komo.reset();
  komo.run();
//  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  moveReba();

  return 0;
}




