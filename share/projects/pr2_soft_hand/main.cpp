#include <Control/gamepad2tasks.h>
#include <Control/taskController.h>
#include <Hardware/gamepad/gamepad.h>
//#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <RosCom/roscom.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <std_msgs/String.h>
#include <RosCom/rosmacro.h>


struct MySystem{
    ACCESS(SoftHandMsg,sh_ref)
  MySystem(){
    new RosCom_Spinner();
    addModule<RosCom_SoftHandSync>(NULL, Module::loopWithBeat,.1);
    //connect();
  }
};

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  MySystem S;
  threadOpenModules(true);

  SoftHandMsg grasp("medium wrap");
  SoftHandMsg deflate("deflate");

  S.sh_ref.set() = grasp;
  mlr::wait(5.);
  S.sh_ref.set() = deflate;

  return 0;

  ors::KinematicWorld world("model.kvg");
  world.gl().resize(800,800);
  world.watch(true);


  MotionProblem MP(world);

  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();

  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

// position task maps
t = MP.addTask("position", new TaskMap_Default(posTMT, world, "endeff_soft_hand", NoVector, "target",NoVector));
t->setCostSpecs(MP.T, MP.T, {0.}, 1e2);

optConstrained(x, NoArr, Convert(MF), OPT(verbose=0,stopTolerance = 1e-3));

displayTrajectory(x,MP.T,world,"traj");

return 0;
}
