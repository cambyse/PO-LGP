#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Gui/opengl.h>
#include <Kin/kin_swift.h>
#include <Kin/taskMaps.h>
#include <Kin/kinViewer.h>
#include <Kin/TM_StaticStability.h>

using namespace std;

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  mlr::KinematicWorld K("kin.g");

  KOMO komo;
  komo.setModel(K);
  komo.world.report();
  komo.setPathOpt(4., 20, 10.);

  komo.setGrasp(1., "baxterL", "red");
  komo.setPlace(2., "baxterL", "red", "yellow");

  komo.setGrasp(2., "baxterR", "yellow");
  komo.setPlace(3., "baxterR", "yellow", "blue");

  komo.setTask(3.5, -1., new TM_StaticStability(komo.world, "blue"));

  komo.reset();
  komo.reportProblem();
  komo.run();
//  komo.checkGradients();
  cout <<komo.getReport(true) <<endl;

  while(komo.displayTrajectory(.1, true));
//  renderConfigurations(komo.configurations, "z.vid/z.path.", -2, 600, 600, &komo.displayCamera());

  return 0;
}

