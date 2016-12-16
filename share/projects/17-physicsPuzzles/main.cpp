#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
//#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>

#include <Motion/komo.h>


#include <Ors/ors_swift.h>

//===========================================================================

void TEST(UsingKomo){
  mlr::KinematicWorld W("model.g");

  KOMO komo;
  komo.setModel(W);

  komo.setTiming(3., 20, 5., 2, true);
  komo.setSquaredFixJointVelocities(-1., -1., 1e2);
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e2);
//  komo.setSquaredQAccelerations();
  komo.setSquaredQVelocities();

  komo.setTask(1., 1., new TaskMap_GJK(W, "ball", "block", true), sumOfSqrTT, NoArr, 1e2);

  double above = .06;
  mlr::Transformation rel;
  rel.addRelativeTranslation(0,above,0);
  komo.setKinematicSwitch(1., true, "delete", "table","ball");
  komo.setKinematicSwitch(1., true, "transXActuated", "table", "ball", rel);

  komo.setTask(2., 2., new TaskMap_Default(posTMT, W, "ball"), sumOfSqrTT, ARR(.5,.06,.5), 1e2, 0);

  komo.setKinematicSwitch(1., false, "delete", "table","ball");
  komo.setKinematicSwitch(1., false, "freeActuated", "table", "ball");

  komo.setTask(3., 3., new TaskMap_Default(posTMT, W, "ball"), sumOfSqrTT, ARR(.8,-.3,.5), 1e2, 0);

  komo.reset();
  komo.run();
//      komo.checkGradients();

  cout <<komo.getReport(true);
  komo.MP->costReport(true);

  komo.displayCamera().setPosition(1.,1.,10.);
  komo.displayCamera().focus(0,0,.5);
  komo.displayCamera().upright(Vector_y);
  while(komo.displayTrajectory(.1, true));
}

//===========================================================================


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  testUsingKomo();

  return 0;
}
