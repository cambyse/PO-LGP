#include <Motion/komo.h>
#include <string>
#include <map>
#include <Gui/opengl.h>

using namespace std;

//===========================================================================

void TEST(KomoSequence){
  
  mlr::KinematicWorld W("model.g");
  KOMO komo;
  komo.setModel(W);
//  komo.setPoseOpt();
//  komo.setSequenceOpt(3.);
  komo.setPathOpt(3.);

  komo.displayCamera().setPosition(-5.,-1.,2.);
  komo.displayCamera().focus(0,0,1.);
  komo.displayCamera().upright();

  komo.setGrasp(1., "humanR", "blue");
//  komo.setPlace(1.8, "humanR", "blue", "tableC");
  komo.setPlace(1.8, "humanR", "blue", "red");

  komo.setGrasp(1.3, "humanL", "yellow");
//  komo.setPlace(2.1, "humanL", "yellow", "tableC");
  komo.setPlace(2.1, "humanL", "yellow", "blue");

  komo.setAlignedStacking(2.2, "yellow");
  komo.setAlignedStacking(2.2, "blue");

  komo.reset();
  komo.run();
//  komo.checkGradients();

  Graph result = komo.getReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha=1.;

  testKomoSequence();

  return 0;
}

