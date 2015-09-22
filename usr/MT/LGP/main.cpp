//#include <pr2/actionMachine.h>
//#include "manipSim.h"
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>



//===========================================================================
int main(int argc,char **argv){
  MT::initCmdLine(argc, argv);
//  rnd.clockSeed();
  rnd.seed(MT::getParameter<int>("seed",0));

  coreExperiment();
//  generateRandomProblems();
//  return 0;
//  optimizeFinal();
//    optimSwitchConfigurations();
//  testReachable();
//  testMonteCarlo();


  return 0;
}
