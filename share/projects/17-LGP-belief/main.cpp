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

  Task *t = komo.setTask(1.5, 1.5, new TaskMap_Default(posTMT, K, "base_footprint"), OT_sumOfSqr, {1., -2., 0.}, 1e1);

//  komo.setTask(1., -1., new TaskMap_Default(gazeAtTMT, K, "endeffEyes", NoVector, "landmark"), OT_sumOfSqr, {}, 1e1);

  komo.setTask(3., -1., new TaskMap_qItself(QIP_byJointNames, {"worldTranslationRotation"}, K), OT_sumOfSqr, q_target);

  komo.setTask(-1., -1., new TaskMap_BeliefTransition(new TaskMap_Default(gazeAtTMT, K, "endeffEyes", NoVector, "landmark")), OT_sumOfSqr, {}, 1e5);
//  komo.setTask(-1., -1., new TaskMap_BeliefTransition(), OT_sumOfSqr, {}, 1e5);

  komo.setTask(4., -1., new TaskMap_qUncertainties(), OT_sumOfSqr);

  komo.reset();
  komo.reportProblem();
  komo.run();
  cout <<komo.getReport(true) <<endl;
//  komo.checkGradients();

  //-----------------------------
  // add collision and belief dynamics

#if 0
  t->prec.clear();
  komo.setCollisions(true, .05, 1e1);
  komo.run();
  cout <<komo.getReport(true) <<endl;
  komo.checkGradients();
#endif

  komo.plotTrajectory();
  while(komo.displayTrajectory(.1, true));
}

int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

  solve();

  return 0;
}

