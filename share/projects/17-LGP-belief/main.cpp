#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

#include <Kin/taskMaps.h>
#include <Kin/frame.h>
#include <Kin/taskMap_BeliefTransition.h>
#include <Kin/taskMap_qUncertainties.h>

#include <KOMO/komo.h>

#include <Kin/taskMap_default.h>

void solve(){
  mlr::KinematicWorld K("kin.g");

  arr q_target = K.getFrameByName("target")->joint()->q0;

  K.report();
  K.setActiveJointsByName({"worldTranslationRotation", "head_pan_joint", "head_tilt_joint"});
  K.report();

  KOMO komo;
  komo.setModel(K);
  komo.world.report();
  komo.setPathOpt(6., 10, 10.);

  //initialization task...
  Task *t = komo.setTask(1.5, 1.5, new TaskMap_Default(posTMT, K, "base_footprint"), OT_sumOfSqr, {1., -2., 0.}, 1e1);

  //gaze at landmark
//  komo.setTask(1., -1., new TaskMap_Default(gazeAtTMT, K, "endeffEyes", NoVector, "landmark"), OT_sumOfSqr, {}, 1e1);

  //3D target pose
  komo.setTask(3., -1., new TaskMap_qItself(QIP_byJointNames, {"worldTranslationRotation"}, K), OT_sumOfSqr, q_target);

  //belief dynamics - modulated by gaze at landmark
  komo.setTask(-1., -1., new TaskMap_BeliefTransition(new TaskMap_Default(gazeAtTMT, K, "endeffEyes", NoVector, "landmark")), OT_sumOfSqr, {}, 1e3);
//  komo.setTask(-1., -1., new TaskMap_BeliefTransition(), OT_sumOfSqr, {}, 1e5);

  //belief target
  komo.setTask(4., -1., new TaskMap_qUncertainties(), OT_sumOfSqr);

  komo.reset();
  komo.reportProblem();
  komo.run();
  cout <<komo.getReport(true) <<endl;
//  komo.checkGradients();

  //-----------------------------
  // add collision and belief dynamics

#if 1
  t->prec.clear();
  komo.setCollisions(false, .05, 1e2);
  komo.run();
  cout <<komo.getReport(true) <<endl;
//  komo.checkGradients();
#endif

  komo.plotTrajectory();
  while(komo.displayTrajectory(.1, true));
}

int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

  solve();

  return 0;
}

