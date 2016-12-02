#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
//#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>

#include <Motion/komo.h>

#include <Ors/ors_swift.h>

//===========================================================================

void TEST(PickAndPlace){
  Graph specs("specsPush.g");
  KOMO komo(specs);
  arr q=komo.MP->world.getJointState();
  komo.MP->world.setJointState(q);
  komo.reset();
  komo.MP->reportFull(true);
  komo.run();
  komo.MP->costReport(true);
  for(;;)
    komo.displayTrajectory();
}

//===========================================================================


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  testPickAndPlace();

  return 0;
}
