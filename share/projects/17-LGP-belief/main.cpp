#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

#include <Kin/taskMaps.h>
#include <Kin/frame.h>

#include <KOMO/komo.h>

void solve(){
  mlr::KinematicWorld K("kin.g");

  K.watch(true);
  return;

  arr q_target = K.getFrameByName("target")->joint()->q0;

  K.setActiveJointsByName({"worldTranslationRotation"});

  KOMO komo;
  komo.setModel(K);
  komo.world.report();
  komo.setPathOpt(4., 10, 10.);

  Task *t = komo.setTask(1.5, 1.5, new TaskMap_Default(posTMT, K, "base_footprint"), OT_sumOfSqr, {1., -2., 0.}, 1e1);

  komo.setTask(3., 4., new TaskMap_qItself(QIP_byJointNames, {"worldTranslationRotation"}, K), OT_sumOfSqr, q_target);

  komo.reset();
  komo.reportProblem();
  komo.run();
  cout <<komo.getReport(true) <<endl;
  komo.checkGradients();

  t->prec.clear();
  komo.setCollisions(true, .05, 1e1);
  komo.run();
  cout <<komo.getReport(true) <<endl;
  komo.checkGradients();


  while(komo.displayTrajectory(.1, true));
}

int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

  solve();

  return 0;
}
