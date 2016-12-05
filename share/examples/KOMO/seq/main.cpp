#include <Motion/komo.h>
#include <string>
#include <map>

using namespace std;

//===========================================================================

void TEST(KomoSequence){
  
  KOMO komo;
  komo.setConfigFromFile();

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchVelocities();
  komo.setSquaredQAccelerations();

  komo.setGrasp(1., "humanR", "Long1");
  komo.setPlace(1.8, "humanR", "Long1", "tableL");
  komo.setSlowAround(1., .1, 1e3);

  komo.setGrasp(1., "humanL", "Long2");
  komo.setPlace(1.8, "humanL", "Long2", "tableR");

  komo.reset();
  komo.run();
//  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha=1.;

  testKomoSequence();

  return 0;
}

