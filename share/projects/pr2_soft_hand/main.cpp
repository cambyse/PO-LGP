#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <Core/array-vector.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>


int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  ors::KinematicWorld world("model.kvg");
  world.gl().resize(800,800);
  world.watch(true);


  MotionProblem MP(world);

  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();

  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  // position task maps
  t = MP.addTask("position", new DefaultTaskMap(posTMT, world, "endeff_soft_hand", NoVector, "target",NoVector));
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e2);

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=0,stopTolerance = 1e-3));

  displayTrajectory(x,MP.T,world,"traj");

  return 0;
}
