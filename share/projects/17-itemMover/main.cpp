#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>


//===========================================================================

void tutorialBasics(){
  mlr::KinematicWorld K("model.g");

  KOMO komo;
  /* there are essentially three things that KOMO needs to be specified:
   * 1) the kinematic model
   * 2) the timing parameters (duration/phases, number os time slices per phase)
   * 3) the tasks */

  //-- setting the model; false -> NOT calling collision detection (SWIFT) -> faster
  komo.setModel(K, false);

  //-- the timing parameters: 1 phase, 20 time slices, 5 seconds, k=2 (acceleration mode)
  komo.setTiming(2, 20, 5., 2);

  //-- default tasks for transition costs
//  komo.setFixEffectiveJoints(); //only relevant when there are kinematic switches
//  komo.setFixSwitchedObjects(); //only relevant when there are kinematic switches
  komo.setSquaredQAccelerations();
  komo.setSquaredQuaternionNorms(-1., -1., 1e3); //when the kinematics includes quaternion joints, keep them roughly regularized

  //-- simple tasks, called low-level

  //in phase-time [1,\infty] position-difference between "endeff" and "target" shall be zero (sumOfSqr objective)
  komo.setTask(1., -1., new TaskMap_Default(posDiffTMT, komo.world, "endeff", NoVector, "target", NoVector));

  //in phase-time [1,\infty] quaternion-difference between "endeff" and "target" shall be zero (sumOfSqr objective)
  komo.setTask(1., -1., new TaskMap_Default(quatDiffTMT, komo.world, "endeff", NoVector,
                                            "target", NoVector));
  //I don't recomment setting quaternion tasks! This is only for testing here. Instead, use alignment tasks as in test/KOMO/komo

  //slow down around phase-time 1. (not measured in seconds, but phase)
  komo.setSlowAround(1., .1, 1e3);

  //-- call the optimizer
  komo.reset();
  komo.run();
  //  komo.checkGradients(); //this checks all gradients of the problem by finite difference
  komo.getReport(true); //true -> plot the cost curves
  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory

  /* next step:
   *
   * Have a look at all the other set*** methods, which all add tasks to the KOMO problem. Look into
   * their implementation: they mainly just call setTask(..) with various TaskMaps.
   *
   * The last three arguments of setTask are important:
   *
   * type allows to define whether this is a sumOfSqr, equality, or inequality task
   *
   * target defines the target value in the task space; {} is interpreted as the zero vector
   *
   * order=0 means that the task is about the position(absolute value) in task space
   * order=1 means that the task is about the velocity in task space
   * order=0 means that the task is about the acceleration in task space
   *
   * For instance, setSquaredQAccelerations sets a tasks about the acceleration in the identity map
   *
   * Next, perhaps learn about all the available taskMaps, or implement new differentiable MappingSuccess
   *
   */
}

//===========================================================================

void tutorialInverseKinematics(){
  /* The only difference is that the timing parameters are set differently and the tranision
   * costs need to be velocities (which is just sumOfSqr of the difference to initialization).
   * All tasks should refer to phase-time 1. Internally, the system still created a banded-diagonal
   * Hessian representation, which is some overhead. It then calles exactly the same constrained optimizers */

  mlr::KinematicWorld K("model.g");

  KOMO komo;
  komo.setModel(K, false);

  //-- the timing parameters: 1 phase, 1 time slice
  komo.setTiming(1, 1);

  //-- default tasks for transition costs
  komo.setSquaredQVelocities();
  komo.setSquaredQuaternionNorms(-1., -1., 1e3); //when the kinematics includes quaternion joints, keep them roughly regularized

  //-- simple tasks, called low-level
  komo.setTask(1., -1., new TaskMap_Default(posDiffTMT, komo.world, "endeff", NoVector, "target", NoVector));
  komo.setTask(1., -1., new TaskMap_Default(quatDiffTMT, komo.world, "endeff", NoVector,
                                            "target", NoVector));

  //-- call the optimizer
  komo.reset();
  komo.run();
  //  komo.checkGradients(); //this checks all gradients of the problem by finite difference
  komo.getReport(); //true -> plot the cost curves
  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory
}

//===========================================================================

void moveItem(){
  mlr::KinematicWorld K("model.g");

  KOMO komo;
  komo.setModel(K, false);
  komo.setTiming(3, 20, 5., 2);

  //-- default tasks for transition costs
  komo.setFixEffectiveJoints(); //only relevant when there are kinematic switches
  komo.setFixSwitchedObjects(); //only relevant when there are kinematic switches
  komo.setSquaredQAccelerations();
  komo.setSquaredQuaternionNorms(-1., -1., 1e3); //when the kinematics includes quaternion joints, keep them roughly regularized

  //position at time=1 20cm above target
  komo.setTask(1., 1., new TaskMap_Default(posDiffTMT, komo.world, "endeff", NoVector, "target", {0,0,.2}));
//  komo.setTask(1., -1., new TaskMap_Default(quatDiffTMT, komo.world, "endeff", NoVector, "target", NoVector));

  //position at time=2 at target
  komo.setTask(2., 2., new TaskMap_Default(posDiffTMT, komo.world, "endeff", NoVector, "target", NoVector));

  //scalar product of endeff_y with target_z = 0 -- until time \infty
  //scalar product of endeff_z with target_z = 0
  komo.setTask(2., -1., new TaskMap_Default(vecAlignTMT, komo.world, "endeff", {0,1,0}, NULL, {0,0,1}));
  komo.setTask(2., -1., new TaskMap_Default(vecAlignTMT, komo.world, "endeff", {0,0,1}, NULL, {0,0,1}));

  //"attach" target to endeff
  komo.setKinematicSwitch(2., true, "delete", NULL, "target");
  komo.setKinematicSwitch(2., true, "ballZero", "endeff", "target");

  //hold still
  komo.setSlowAround(2., .1, 1e3);

  //lift: a sos velocity task (order=1)
  komo.setTask(2., -1., new TaskMap_Default(posDiffTMT, komo.world, "endeff"), OT_sumOfSqr, {0., 0., .1}, 1e1, 1);

  //-- call the optimizer
  komo.reset();
  komo.run();
  //  komo.checkGradients(); //this checks all gradients of the problem by finite difference
  komo.getReport(false); //true -> plot the cost curves
  while(komo.displayTrajectory(.05, true)); //play the trajectory
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

//  tutorialBasics();
//  tutorialInverseKinematics();

  moveItem();

  return 0;
}
