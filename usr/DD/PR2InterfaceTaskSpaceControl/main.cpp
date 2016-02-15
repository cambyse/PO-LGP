#include <Core/array.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/motion.h>
#include <pr2/roscom.h>
#include <Core/util.h>
#include <Gui/plot.h>

#include <Hardware/gamepad/gamepad.h>

#include <pr2/pr2Interface.h>
#include <Control/taskSpaceController.h>

void changeAlpha(void*){  orsDrawAlpha = 1.; }

arr generateBrezel() {
  arr brezel;
  brezel.append(ARR(0.65,0.0,0.56));
  brezel.append(ARR(0.65,-0.2,0.56));
  brezel.append(ARR(0.5,-0.3,0.56));
  brezel.append(ARR(0.4,-0.1,0.56));
  brezel.append(ARR(0.5,0.0,0.56));
  brezel.append(ARR(0.6,-0.1,0.56));
  brezel.append(ARR(0.6,0.1,0.56));
  brezel.append(ARR(0.5,0.0,0.56));
  brezel.append(ARR(0.4,0.1,0.56));
  brezel.append(ARR(0.5,0.3,0.56));
  brezel.append(ARR(0.65,0.2,0.56));
  brezel.append(ARR(0.65,0.0,0.56));
  brezel.reshape(brezel.N/3,3);

  return brezel;
}

void followTrajectory() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);  
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();



  //Go from actual position to the start position of the trajectory
  arr trajStart = ARR(0.65,0.0,0.56);

  mlr::Array<LinTaskSpaceAccLaw*> laws;
  TaskMap* posTaskl = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLawl = new LinTaskSpaceAccLaw(posTaskl, modelWorld, "endeffLPos");
  posLawl->setRef(trajStart);
  laws.append(posLawl);

  TaskMap* orientationMapl = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLawl = new LinTaskSpaceAccLaw(orientationMapl, modelWorld, "endeffLOrientation");
  orientationLawl->setRef(ARR(0.0,0.0,-1.0));
  laws.append(orientationLawl);

  pr2->goToTasks(laws);

  modelWorld->watch(true, "Press enter for task space control");

  //pr2->~PR2Interface();


  //mlr::wait(3.0);

  arr posTrajectory = generateBrezel();

  mlr::Spline* spline = new mlr::Spline(posTrajectory.d0, posTrajectory);
  arr sp;
  uint n = 100;
  for(uint i = 0; i <= n; i++) {
    sp.append(spline->eval((double)i/n));
  }
  sp.reshape(n+1,3);
  plotLine(sp);


  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  posLaw->setTrajectory(posTrajectory.d0, posTrajectory);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*30.0;
  posLaw->setGains(Kp,eye(3)*5.0);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory;
  orientationTrajectory.append(~orientationLaw->getPhi());
  orientationTrajectory.append(~ARR(0.0,0.0,-1.0));
  orientationTrajectory.append(~ARR(0.0,0.0,-1.0));
  orientationLaw->setTrajectory(orientationTrajectory.d0, orientationTrajectory);

  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setTrajectory(3,zeros(3,1));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);

  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(30.0);
  mlr::wait(1.0);
  //pr2->logStateSave();
  modelWorld->watch(true, "Press to stop");

  pr2->~PR2Interface();

  /*

  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  arr actState = posLaw->getPhi();
  arr traj;
  traj.append(~actState);
  traj.append(~trajStart);
  traj.append(~trajStart);
  posLaw->setTrajectory(traj.d0, traj);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*10.0;
  posLaw->setGains(Kp,eye(3)*5.0);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory;
  orientationTrajectory.append(~orientationLaw->getPhi());
  orientationTrajectory.append(~ARR(0.0,0.0,-1.0));
  orientationTrajectory.append(~ARR(0.0,0.0,-1.0));
  orientationLaw->setTrajectory(orientationTrajectory.d0, orientationTrajectory);

  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setTrajectory(3,zeros(3,1));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*5.0);
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);

  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(10.0);
  realWorldSimulation->watch(true);

  pr2->~PR2Interface();*/

}

void qDotRefInConstraint() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  //Go from actual position to the start position of the trajectory
  arr trajStart = ARR(0.7,0.0,0.55);

  mlr::Array<LinTaskSpaceAccLaw*> laws;
  TaskMap* posTaskl = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLawl = new LinTaskSpaceAccLaw(posTaskl, modelWorld, "endeffLPos");
  posLawl->setRef(trajStart);
  laws.append(posLawl);

  TaskMap* orientationMapl = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLawl = new LinTaskSpaceAccLaw(orientationMapl, modelWorld, "endeffLOrientation");
  orientationLawl->setRef(ARR(0.0,0.0,-1.0));
  laws.append(orientationLawl);

  pr2->goToTasks(laws);

  modelWorld->watch(true, "press enter for task space control");

  arr posTrajectory;
  posTrajectory.append(~trajStart);
  posTrajectory.append(~trajStart);
  posTrajectory.append(~trajStart);

  mlr::Spline* spline = new mlr::Spline(posTrajectory.d0, posTrajectory);
  arr sp;
  uint n = 100;
  for(uint i = 0; i <= n; i++) {
    sp.append(spline->eval((double)i/n));
  }
  sp.reshape(n+1,3);
  plotLine(sp);


  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  posLaw->setTrajectory(posTrajectory.d0, posTrajectory);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*10.0;
  Kp(2,2) = 0.0;
  arr Kd = eye(3)*5.0;
  Kd(2,2) = 0.0;
  posLaw->setGains(Kp,Kd);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory;
  orientationTrajectory.append(~orientationLaw->getPhi());
  orientationTrajectory.append(~ARR(0.0,0.0,-1.0));
  orientationTrajectory.append(~ARR(0.0,0.0,-1.0));
  orientationLaw->setTrajectory(orientationTrajectory.d0, orientationTrajectory);

  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setTrajectory(3,zeros(3,1));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));

  TaskMap* velMap = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* velLaw = new LinTaskSpaceAccLaw(velMap, modelWorld, "vel");
  velLaw->setC(eye(3)*1000.0);
  arr KdVel = zeros(3,3);
  KdVel(2,2) = 40.0;
  velLaw->setGains(eye(3)*0.0, KdVel);

  arr velTraj = repmat(~ARR(0.0,0.0,-.05), 3, 1);
  velLaw->setTrajectory(3, NoArr, velTraj);

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);
  controller->addLinTaskSpaceAccLaw(velLaw);

  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(10.0);
  mlr::wait(0.5);
  pr2->logState = false;
  pr2->logStateSave("touchdown_8");
  modelWorld->watch(true, "Press to stop");

  pr2->~PR2Interface();
}

void qDotRefInConstraintAndSlide() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");//new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  //Go from actual position to the start position of the trajectory
  arr trajStart = ARR(0.7,0.0,0.6);

  mlr::Array<LinTaskSpaceAccLaw*> laws;
  TaskMap* posTaskl = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLawl = new LinTaskSpaceAccLaw(posTaskl, modelWorld, "endeffLPos");
  posLawl->setRef(trajStart);
  laws.append(posLawl);

  TaskMap* orientationMapl = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLawl = new LinTaskSpaceAccLaw(orientationMapl, modelWorld, "endeffLOrientation");
  orientationLawl->setRef(ARR(0.0,0.0,-1.0));
  laws.append(orientationLawl);

  pr2->goToTasks(laws);

  modelWorld->watch(true, "press enter for task space control");

  arr posTrajectory;
  posTrajectory.append(~trajStart);
  posTrajectory.append(~trajStart);
  posTrajectory.append(~trajStart);

  mlr::Spline* spline = new mlr::Spline(posTrajectory.d0, posTrajectory);
  arr sp;
  uint n = 100;
  for(uint i = 0; i <= n; i++) {
    sp.append(spline->eval((double)i/n));
  }
  sp.reshape(n+1,3);
  plotLine(sp);


  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  posLaw->setTrajectory(posTrajectory.d0, posTrajectory);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*10.0;
  Kp(2,2) = 0.0;
  arr Kd = eye(3)*5.0;
  Kd(2,2) = 0.0;
  posLaw->setGains(Kp,Kd);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory;
  orientationTrajectory.append(~orientationLaw->getPhi());
  orientationTrajectory.append(~ARR(0.0,0.0,-1.0));
  orientationTrajectory.append(~ARR(0.0,0.0,-1.0));
  orientationLaw->setTrajectory(orientationTrajectory.d0, orientationTrajectory);

  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setTrajectory(3,zeros(3,1));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));

  /*TaskMap* velMap = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* velLaw = new LinTaskSpaceAccLaw(velMap, modelWorld, "vel");
  velLaw->setC(eye(3)*1000.0);
  arr KdVel = zeros(3,3);
  KdVel(2,2) = 40.0;
  velLaw->setGains(eye(3)*0.0, KdVel);

  arr velTraj = repmat(~ARR(0.0,0.0,-.05), 3, 1);
  */

  TaskMap* velMap = new DefaultTaskMap(pos1DTMT, *modelWorld, "endeffL", ors::Vector(.0,0.0,-1.0));
  ConstrainedTaskLaw* velLaw = new ConstrainedTaskLaw(velMap, modelWorld, "vel");
  velLaw->setC(eye(1)*1000.0);
  velLaw->setGains(eye(1)*0.0, eye(1)*15.0);
  velLaw->setForce(ARR(-2.0));
  velLaw->setAlpha(ARR(0.0005));
  //velLaw->setAlpha(ARR(0.0));
  controller->constrainedTaskLaw = velLaw;
  arr velTraj = repmat(ARR(.1), 3, 1);

  velLaw->setTrajectory(3, NoArr, velTraj);

 /* TaskMap* gazeAtMap = new DefaultTaskMap(gazeAtTMT, *modelWorld, "endeffKinect", NoVector, "endeffL");
  LinTaskSpaceAccLaw* gazeAtLaw = new LinTaskSpaceAccLaw(gazeAtMap, modelWorld);
  gazeAtLaw->setC(eye(2)*1000.0);
  gazeAtLaw->setGains(eye(2)*10.0,eye(2)*5.0);
  gazeAtLaw->setTrajectory(3, zeros(3,2));
*/
  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);
  controller->addLinTaskSpaceAccLaw(velLaw);
  //controller->addLinTaskSpaceAccLaw(gazeAtLaw);

  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(20.0);
  mlr::wait(0.5);

  /*arr slideTraj;
  slideTraj.append(~ARR(0.7,0.0,0.55));
  slideTraj.append(~ARR(0.7,0.15,0.55));
  slideTraj.append(~ARR(0.7,0.3,0.55));

  posLaw->setTrajectory(slideTraj.d0, slideTraj);

  controller->generateTaskSpaceSplines();
  pr2->executeTrajectory(10.0);

*/
  pr2->logState = false;
  pr2->logStateSave("touchdownAndSlide_15");
  modelWorld->watch(true, "Press to stop");

  pr2->~PR2Interface();
}

void openSchublade() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");//new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  arr posTrajectory;
  posTrajectory.append(~posLaw->getPhi());
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker5")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker5")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker5")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker6")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker6")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker5")->X.pos));
  posLaw->setTrajectory(posTrajectory.d0, posTrajectory);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*10.0;
  Kp(2,2) = 10.0;
  arr Kd = eye(3)*5.0;
  Kd(2,2) = 5.0;
  posLaw->setGains(Kp,Kd);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory;
  orientationTrajectory.append(~orientationLaw->getPhi());
  orientationTrajectory.append(~ARR(1.0,0.0,0.0));
  orientationTrajectory.append(~ARR(1.0,0.0,0.0));
  orientationLaw->setTrajectory(orientationTrajectory.d0, orientationTrajectory);

  TaskMap* orientationMap2 = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(0.,1.,0.));
  LinTaskSpaceAccLaw* orientationLaw2 = new LinTaskSpaceAccLaw(orientationMap2, modelWorld, "endeffLOrientation2");
  orientationLaw2->setC(eye(3)*1000.0);
  orientationLaw2->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory2;
  orientationTrajectory2.append(~orientationLaw2->getPhi());
  orientationTrajectory2.append(~ARR(0.0,0.0,1.0));
  orientationTrajectory2.append(~ARR(0.0,0.0,1.0));
  orientationLaw2->setTrajectory(orientationTrajectory2.d0, orientationTrajectory2);


  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setTrajectory(3,zeros(3,1));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw2);
  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(20.0);
  mlr::wait(0.5);
  pr2->modelWorld->watch(true);
}


void testTorsoLiftLink() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");//new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  arr q = realWorld->getJointState();
  cout << q(realWorld->getJointByName("l_gripper_joint")->qIndex) << endl;

  realWorld->watch(true, "press to openGripper");
  pr2->moveTorsoLift(ARR(0.00));
  //pr2->moveLGripper(ARR(0.055));
  realWorld->watch(true, "press to stop");

  pr2->~PR2Interface();

}



void openSchublade2() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");//new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();


 /* TaskMap* gazeAtMap = new DefaultTaskMap(gazeAtTMT, *modelWorld, "endeffKinect", NoVector, "endeffL");
  LinTaskSpaceAccLaw* gazeAtLaw = new LinTaskSpaceAccLaw(gazeAtMap, modelWorld);
  gazeAtLaw->setC(eye(2)*1000.0);
  gazeAtLaw->setGains(eye(2)*10.0,eye(2)*5.0);
  gazeAtLaw->setRef(ARR(0.0,0.0));

  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*1.0;
  Kp(2,2) = 1.0;
  arr Kd = eye(3)*1.0;
  Kd(2,2) = 1.0;
  posLaw->setGains(Kp,Kd);
  posLaw->setRef();

  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(gazeAtLaw);

  modelWorld->watch(true,"wait for marker to be found");

  ors::Transformation markerSchublade = modelWorld->getShapeByName("marker2")->X;


  arr markerSchubladePos = conv_vec2arr(markerSchublade.pos);

  cout << markerSchubladePos << endl;

  modelWorld->watch(true,"note marker");
*/

  arr markerSchubladePos = ARR(0.6, 0.2,.7);//ARR(0.904966, 0.0657443, 1.04619);

  //markerSchubladePos(0) -= 0.1;
  //markerSchubladePos(2) -= 0.01;

  mlr::Array<LinTaskSpaceAccLaw*> laws;
  TaskMap* posTaskl = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLawl = new LinTaskSpaceAccLaw(posTaskl, modelWorld, "endeffLPos");
  posLawl->setRef(markerSchubladePos);
  laws.append(posLawl);

  TaskMap* orientationMapl = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLawl = new LinTaskSpaceAccLaw(orientationMapl, modelWorld, "endeffLOrientation");
  orientationLawl->setRef(ARR(1.0,0.0,0.0));
  laws.append(orientationLawl);

  TaskMap* orientationMapl2 = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(0.,1.,0.));
  LinTaskSpaceAccLaw* orientationLawl2 = new LinTaskSpaceAccLaw(orientationMapl2, modelWorld, "endeffLOrientation");
  orientationLawl2->setRef(ARR(0.0,0.0,1.0));
  //laws.append(orientationLawl2);


  pr2->goToTasks(laws);

  modelWorld->watch(true, "press enter for task space control");






  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  arr posTrajectory;
  posTrajectory.append(~posLaw->getPhi());
  posTrajectory.append(~posLaw->getPhi());
  posTrajectory.append(~posLaw->getPhi());
  posLaw->setTrajectory(posTrajectory.d0, posTrajectory);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*20.0;
  Kp(1,1) = 10.0;
  Kp(0,0) = 0.0;
  arr Kd = eye(3)*5.0;
  Kd(1,1) = 5.0;
  Kd(0,0) = 0.0;
  posLaw->setGains(Kp,Kd);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*30.0,eye(3)*5.0);
  arr orientationTrajectory;
  orientationTrajectory.append(~orientationLaw->getPhi());
  orientationTrajectory.append(~ARR(1.0,0.0,0.0));
  orientationTrajectory.append(~ARR(1.0,0.0,0.0));
  orientationLaw->setTrajectory(orientationTrajectory.d0, orientationTrajectory);

  TaskMap* orientationMap2 = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(0.,1.,0.));
  LinTaskSpaceAccLaw* orientationLaw2 = new LinTaskSpaceAccLaw(orientationMap2, modelWorld, "endeffLOrientation2");
  orientationLaw2->setC(eye(3)*1000.0);
  orientationLaw2->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory2;
  orientationTrajectory2.append(~orientationLaw2->getPhi());
  orientationTrajectory2.append(~ARR(0.0,0.0,1.0));
  orientationTrajectory2.append(~ARR(0.0,0.0,1.0));
  orientationLaw2->setTrajectory(orientationTrajectory2.d0, orientationTrajectory2);




  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setTrajectory(3,zeros(3,1));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));


  TaskMap* velMap = new DefaultTaskMap(pos1DTMT, *modelWorld, "endeffL", ors::Vector(1.0,0.0,.0));
  ConstrainedTaskLaw* velLaw = new ConstrainedTaskLaw(velMap, modelWorld, "vel");
  velLaw->setC(eye(1)*1000.0);
  velLaw->setGains(eye(1)*0.0, eye(1)*15.0);
  velLaw->setForce(ARR(-2.0));
  //velLaw->setAlpha(ARR(0.0005));
  velLaw->setAlpha(ARR(0.0));
  controller->constrainedTaskLaw = velLaw;
  arr velTraj = repmat(ARR(0.1), 3, 1);

  velLaw->setTrajectory(3, NoArr, velTraj);

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw2);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);
  controller->addLinTaskSpaceAccLaw(velLaw);

  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(20.0);




  /*TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  arr posTrajectory;
  posTrajectory.append(~posLaw->getPhi());
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker5")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker5")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker5")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker6")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker6")->X.pos));
  posTrajectory.append(~conv_vec2arr(modelWorld->getShapeByName("marker5")->X.pos));
  posLaw->setTrajectory(posTrajectory.d0, posTrajectory);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*10.0;
  Kp(2,2) = 10.0;
  arr Kd = eye(3)*5.0;
  Kd(2,2) = 5.0;
  posLaw->setGains(Kp,Kd);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory;
  orientationTrajectory.append(~orientationLaw->getPhi());
  orientationTrajectory.append(~ARR(1.0,0.0,0.0));
  orientationTrajectory.append(~ARR(1.0,0.0,0.0));
  orientationLaw->setTrajectory(orientationTrajectory.d0, orientationTrajectory);

  TaskMap* orientationMap2 = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(0.,1.,0.));
  LinTaskSpaceAccLaw* orientationLaw2 = new LinTaskSpaceAccLaw(orientationMap2, modelWorld, "endeffLOrientation2");
  orientationLaw2->setC(eye(3)*1000.0);
  orientationLaw2->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory2;
  orientationTrajectory2.append(~orientationLaw2->getPhi());
  orientationTrajectory2.append(~ARR(0.0,0.0,1.0));
  orientationTrajectory2.append(~ARR(0.0,0.0,1.0));
  orientationLaw2->setTrajectory(orientationTrajectory2.d0, orientationTrajectory2);


  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setTrajectory(3,zeros(3,1));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw2);
  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(20.0);
  mlr::wait(0.5);
  */
  pr2->modelWorld->watch(true, "press to stop");
  pr2->~PR2Interface();
}



void testDemonstration() {

  ACCESS(arr, gamepadState);
  GamepadInterface gamepad;

  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");//new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  controller->taskSpaceAccLaws.clear();

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setRef(zeros(qDampingLaw->getPhiDim()), zeros(qDampingLaw->getPhiDim()));

  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  modelWorld->watch(true, "press to start");

  while(true){
    gamepadState.var->waitForNextRevision();
    arr state = gamepadState.get()();
    if(state(0) == 1) {
      mlr::wait(1.0);
      write(LIST<arr>(~modelWorld->getJointState()), STRING("demonstrationData/demonstration.dat"));
      break;
    }
    if(moduleShutdown().getValue()) break;
  }

  //mlr::wait(20.0);
  pr2->modelWorld->watch(true, "press to stop");
  pr2->~PR2Interface();
}


void openSchublade3() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");//new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  //arr startPos = ARR(0.135744,0.0244433,-0.925246,0.770287,-0.388091,0.222636, 0.549529,0.0191242);
  //startPos.append(ARR(1.55055,-0.154154,-1.76661,0.115404,-2.61978,-0.0870273,-1.09837,-0.0345408,0.0920215));

  arr startPos = ARR( 0.135535, 0.025378, -0.911152, 0.709683, -0.388196, 0.0506541, 0.84, 0.285634);//0.781742
  startPos.append(ARR(1.44696, -0.175291, -1.4678, -1.86603, -2.31562, -0.404729, -0.706922, 0.274025, 0.0184479));

  TaskMap* qMap = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qLaw = new LinTaskSpaceAccLaw(qMap, modelWorld, "qMap");
  qLaw->setRef(startPos);
  mlr::Array<LinTaskSpaceAccLaw*> laws;
  laws.append(qLaw);
  pr2->goToTasks(laws);

  pr2->moveLGripper(ARR(0.02));
  modelWorld->watch(true, "press enter for task space control");






  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  arr posTrajectory;
  posTrajectory.append(~posLaw->getPhi());
  posTrajectory.append(~posLaw->getPhi());
  posTrajectory.append(~posLaw->getPhi());
  posLaw->setTrajectory(posTrajectory.d0, posTrajectory);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*20.0;
  Kp(1,1) = 10.0;
  Kp(0,0) = 0.0;
  arr Kd = eye(3)*5.0;
  Kd(1,1) = 5.0;
  Kd(0,0) = 0.0;
  posLaw->setGains(Kp,Kd);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
  arr orientationTrajectory;
  orientationTrajectory.append(~orientationLaw->getPhi());
  orientationTrajectory.append(~ARR(1.0,0.0,0.0));
  orientationTrajectory.append(~ARR(1.0,0.0,0.0));
  orientationLaw->setTrajectory(orientationTrajectory.d0, orientationTrajectory);

  TaskMap* orientationMap2 = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(0.,1.,0.));
  LinTaskSpaceAccLaw* orientationLaw2 = new LinTaskSpaceAccLaw(orientationMap2, modelWorld, "endeffLOrientation2");
  orientationLaw2->setC(eye(3)*1000.0);
  orientationLaw2->setGains(eye(3)*10.0,eye(3)*3.0);
  arr orientationTrajectory2;
  orientationTrajectory2.append(~orientationLaw2->getPhi());
  orientationTrajectory2.append(~ARR(0.0,0.0,-1.0));
  orientationTrajectory2.append(~ARR(0.0,0.0,-1.0));
  orientationLaw2->setTrajectory(orientationTrajectory2.d0, orientationTrajectory2);




  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setTrajectory(3,zeros(3,1));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld);
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));


  TaskMap* velMap = new DefaultTaskMap(pos1DTMT, *modelWorld, "endeffL", ors::Vector(1.0,0.0,.0));
  ConstrainedTaskLaw* velLaw = new ConstrainedTaskLaw(velMap, modelWorld, "vel");
  velLaw->setC(eye(1)*1000.0);
  velLaw->setGains(eye(1)*0.0, eye(1)*9.0);
  velLaw->setForce(ARR(-3.5));
  velLaw->setAlpha(ARR(0.0008));
  //velLaw->setAlpha(ARR(0.0));
  velLaw->gamma = 1.0;
  controller->constrainedTaskLaw = velLaw;
  arr velTraj = repmat(ARR(0.1), 3, 1);

  velLaw->setTrajectory(3, NoArr, velTraj);

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw2);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);
  controller->addLinTaskSpaceAccLaw(velLaw);

  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(7.0);

  Kp(2,2) = 4.0;
  Kp(1,1) = 4.0;
  Kd(2,2) = 2.0;
  Kd(1,1) = 2.0;
  posLaw->setGains(Kp,Kd);
  orientationLaw2->setGains(eye(3)*0.5,eye(3)*0.0);
  //orientationLaw2->setC(eye(3)*0.0);
  orientationLaw->setGains(eye(3)*3.0,eye(3)*1.0);
  pr2->moveLGripper(ARR(0.002));
  mlr::wait(3.0);

  velTraj = repmat(ARR(-0.12), 3, 1);

  velLaw->setTrajectory(3, NoArr, velTraj);
  velLaw->setGains(eye(1)*0.0, eye(1)*10.0);
  velLaw->setAlpha(ARR(0.0));
  velLaw->gamma = 0.0;
  controller->generateTaskSpaceSplines();
  velLaw->gamma = 1.0;
  pr2->executeTrajectory(4.0);

  velLaw->setGains(eye(1)*0.0, eye(1)*0.0);

  pr2->moveLGripper(ARR(0.03));
  velLaw->gamma = 0.0;
  mlr::wait(2.0);
  pr2->logState = false;
  pr2->logStateSave("openSchublade_35");

  modelWorld->watch(true, "press to stop");
  pr2->~PR2Interface();

}


void controllerExample(mlr::String mode) {

  ACCESS(arr, gamepadState);
  GamepadInterface gamepad;

  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  if(mode == "nullSpace_1") {
    TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
    LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
    posLaw->setC(eye(3)*1000.0);
    arr Kp = eye(3)*10.0;
    arr Kd = eye(3)*5.0;
    posLaw->setGains(Kp,Kd);
    posLaw->setRef(posLaw->getPhi());

    TaskMap_qLimits* lmap = new TaskMap_qLimits();
    LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
    limitsLaw->setC(ARR(1000.0));
    limitsLaw->setGains(ARR(10.0),ARR(5.0));
    limitsLaw->setRef(ARR(0.0));

    controller->taskSpaceAccLaws.clear();
    controller->addLinTaskSpaceAccLaw(posLaw);
    controller->addLinTaskSpaceAccLaw(limitsLaw);

    while(true){
      gamepadState.var->waitForNextRevision();
      arr state = gamepadState.get()();
      if(state(0) == 1) {
        posLaw->setGains(zeros(3,3), zeros(3,3));
      } else if(state(0) == 2) {
        posLaw->setRef(posLaw->getPhi());
        posLaw->setGains(Kp,Kd);
      }
      if(moduleShutdown().getValue()) break;
    }
  }

  if(mode == "nullSpace_2") {
    TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
    LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
    posLaw->setC(eye(3)*1000.0);
    arr Kp = eye(3)*10.0;
    arr Kd = eye(3)*5.0;
    posLaw->setGains(Kp,Kd);
    posLaw->setRef(posLaw->getPhi());

    TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
    LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
    orientationLaw->setC(eye(3)*1000.0);
    orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
    orientationLaw->setRef(orientationLaw->getPhi());

    TaskMap_qLimits* lmap = new TaskMap_qLimits();
    LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
    limitsLaw->setC(ARR(1000.0));
    limitsLaw->setGains(ARR(10.0),ARR(5.0));
    limitsLaw->setRef(ARR(0.0));

    controller->taskSpaceAccLaws.clear();
    controller->addLinTaskSpaceAccLaw(posLaw);
    controller->addLinTaskSpaceAccLaw(orientationLaw);
    controller->addLinTaskSpaceAccLaw(limitsLaw);

    while(true){
      gamepadState.var->waitForNextRevision();
      arr state = gamepadState.get()();
      if(state(0) == 1) {
        posLaw->setGains(zeros(3,3), zeros(3,3));
        orientationLaw->setGains(zeros(3,3), zeros(3,3));
      } else if(state(0) == 2) {
        posLaw->setRef(posLaw->getPhi());
        posLaw->setGains(Kp,Kd);
        orientationLaw->setRef(orientationLaw->getPhi());
        orientationLaw->setGains(Kp, Kd);
      }
      if(moduleShutdown().getValue()) break;
    }
  }

  if(mode == "differentTaskSpaceStiffness_1") {
    TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
    LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
    posLaw->setC(eye(3)*1000.0);
    arr Kp = eye(3)*10.0;
    arr Kd = eye(3)*5.0;
    posLaw->setGains(Kp,Kd);
    posLaw->setRef(posLaw->getPhi());

    TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
    LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
    orientationLaw->setC(eye(3)*1000.0);
    orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
    orientationLaw->setRef(orientationLaw->getPhi());

    TaskMap_qLimits* lmap = new TaskMap_qLimits();
    LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
    limitsLaw->setC(ARR(1000.0));
    limitsLaw->setGains(ARR(10.0),ARR(5.0));
    limitsLaw->setRef(ARR(0.0));


    controller->taskSpaceAccLaws.clear();
    controller->addLinTaskSpaceAccLaw(posLaw);
    controller->addLinTaskSpaceAccLaw(orientationLaw);
    controller->addLinTaskSpaceAccLaw(limitsLaw);

    while(true){
      gamepadState.var->waitForNextRevision();
      arr state = gamepadState.get()();
      if(state(0) == 1) {
        posLaw->setGains(zeros(3,3), zeros(3,3));
        orientationLaw->setGains(zeros(3,3), zeros(3,3));
      } else if(state(0) == 2) {
        Kp = eye(3)*10.0;
        Kd = eye(3)*5.0;
        posLaw->setRef(posLaw->getPhi());
        posLaw->setGains(Kp,Kd);
        orientationLaw->setRef(orientationLaw->getPhi());
        orientationLaw->setGains(Kp, Kd);
      } else if(state(0) == 4) {
        Kp(0,0) = 0.0;
        Kd(0,0) = 0.5;
        Kp(1,1) = 10.0;
        Kd(1,1) = 5.0;
        Kp(2,2) = 10.0;
        Kd(2,2) = 5.0;
        posLaw->setGains(Kp,Kd);
      } else if(state(0) == 8) {
        Kp(0,0) = 10.0;
        Kd(0,0) = 5.0;
        Kp(1,1) = 0.0;
        Kd(1,1) = 1.0;
        Kp(2,2) = 10.0;
        Kd(2,2) = 5.0;
        posLaw->setGains(Kp,Kd);
      }
      if(moduleShutdown().getValue()) break;
    }
  }

  modelWorld->watch(true, "press to stop");
  pr2->~PR2Interface();

}


void tischTouchdown_1() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");//new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  //Go from actual position to the start position of the trajectory
  arr preTouchPos = ARR(0.7,0.0,0.6);

  mlr::Array<LinTaskSpaceAccLaw*> laws;
  TaskMap* posTaskl = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLawl = new LinTaskSpaceAccLaw(posTaskl, modelWorld, "endeffLPos");
  posLawl->setRef(preTouchPos);
  laws.append(posLawl);

  TaskMap* orientationMapl = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLawl = new LinTaskSpaceAccLaw(orientationMapl, modelWorld, "endeffLOrientation");
  orientationLawl->setRef(ARR(0.0,0.0,-1.0));
  laws.append(orientationLawl);

  pr2->goToTasks(laws);

  pr2->logState = false;
  pr2->clearLog();
  pr2->logState = true;
  modelWorld->watch(true, "press enter for task space control");


  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*10.0;
  Kp(2,2) = 0.0;
  arr Kd = eye(3)*5.0;
  Kd(2,2) = 0.0;
  posLaw->setGains(Kp,Kd);
  posLaw->setRef(preTouchPos);

  TaskMap* orientationMap = new DefaultTaskMap(vecTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLaw = new LinTaskSpaceAccLaw(orientationMap, modelWorld, "endeffLOrientation");
  orientationLaw->setC(eye(3)*1000.0);
  orientationLaw->setGains(eye(3)*10.0,eye(3)*5.0);
  orientationLaw->setRef(ARR(0.0,0.0,-1.0));

  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  limitsLaw->setRef(ARR(0.0));

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld, "damping");
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  qDampingLaw->setRef(zeros(qDampingLaw->getPhiDim()),zeros(qDampingLaw->getPhiDim()));

  TaskMap* velMap = new DefaultTaskMap(pos1DTMT, *modelWorld, "endeffL", ors::Vector(.0,0.0,-1.0));
  ConstrainedTaskLaw* velLaw = new ConstrainedTaskLaw(velMap, modelWorld, "qDotRefInConstraint");
  velLaw->setC(eye(1)*1000.0);
  velLaw->setGains(eye(1)*0.0, eye(1)*20.0);
  velLaw->setForce(ARR(-0.1));
  //velLaw->setAlpha(ARR(0.001));
  velLaw->setAlpha(ARR(0.0));
  velLaw->gamma = 1.0;
  controller->constrainedTaskLaw = velLaw;
  velLaw->setRef(NoArr, ARR(0.1));

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);
  controller->addLinTaskSpaceAccLaw(velLaw);

  mlr::wait(0.5);

  modelWorld->watch(true, "Press to save log");

  pr2->logState = false;
  pr2->logStateSave("tischTouchdown_1_7","experiments/tischTouchdown_1/");


  modelWorld->watch(true, "Press to stop");
  velLaw->gamma = 0.0;

  pr2->~PR2Interface();
}

void tests() {
  ors::KinematicWorld* modelWorld = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");//new ors::KinematicWorld("pr2_model_for_tasks/pr2_model_for_tasks.ors");
  ors::KinematicWorld* realWorld = new ors::KinematicWorld("pr2_model/pr2_model.ors");
  ors::KinematicWorld* realWorldSimulation = new ors::KinematicWorld("pr2_model_for_simulation/pr2_model_for_simulation.ors");

  modelWorld->gl().add(changeAlpha);
  modelWorld->gl().add(glDrawPlot, &plotModule);

  realWorld->gl().add(changeAlpha);
  realWorldSimulation->gl().add(changeAlpha);

  PR2Interface* pr2 = new PR2Interface();
  TaskSpaceController* controller = new TaskSpaceController(modelWorld);

  pr2->initialize(realWorld, realWorldSimulation, modelWorld, controller);
  pr2->startInterface();

  //pr2->logStateSave("bla","tests/bla/");

  arr preTouchPos = ARR(0.7,0.0,0.6);

  mlr::Array<LinTaskSpaceAccLaw*> laws;
  TaskMap* posTaskl = new DefaultTaskMap(pos1DTMT, *modelWorld, "endeffL", ors::Vector(1/sqrt(2),1/sqrt(2),0.0));
  LinTaskSpaceAccLaw* posLawl = new LinTaskSpaceAccLaw(posTaskl, modelWorld, "endeffLPos");
  posLawl->setC(eye(1)*1000.0);
  posLawl->setGains(eye(1)*10.0, eye(1)*5.0);
  //posLawl->setRef(ARR(0.0));
  //laws.append(posLawl);

  arr traj = ARR(posLawl->getPhi(),0.0,0.0);
  traj.d1 = 1;
  posLawl->setTrajectory(3,traj);

  /*TaskMap* orientationMapl = new DefaultTaskMap(quatTMT, *modelWorld,"endeffL",ors::Vector(1.,0.,0.));
  LinTaskSpaceAccLaw* orientationLawl = new LinTaskSpaceAccLaw(orientationMapl, modelWorld, "endeffLOrientation");
  orientationLawl->setC(eye(4)*1000.0);
  orientationLawl->setGains(eye(4)*10.0, eye(4)*5.0);
  orientationLawl->setRef(ARR(1.0,0.0,1.0,0.0));
  laws.append(orientationLawl);
  */
  TaskMap_qLimits* lmap = new TaskMap_qLimits();
  LinTaskSpaceAccLaw* limitsLaw = new LinTaskSpaceAccLaw(lmap, modelWorld, "limits");
  limitsLaw->setC(ARR(1000.0));
  limitsLaw->setGains(ARR(10.0),ARR(5.0));
  //limitsLaw->setRef(ARR(0.0));
  limitsLaw->setTrajectory(3,zeros(3,1));
  laws.append(limitsLaw);

  TaskMap* qDamping = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qDampingLaw = new LinTaskSpaceAccLaw(qDamping, modelWorld, "damping");
  qDampingLaw->setC(eye(qDampingLaw->getPhiDim())*10.0);
  qDampingLaw->setGains(zeros(qDampingLaw->getPhiDim(),qDampingLaw->getPhiDim()), eye(qDampingLaw->getPhiDim())*1.0);
  //qDampingLaw->setRef(zeros(qDampingLaw->getPhiDim()),zeros(qDampingLaw->getPhiDim()));
  qDampingLaw->setTrajectory(3,zeros(3,qDampingLaw->getPhiDim()), zeros(3,qDampingLaw->getPhiDim()));

  laws.append(qDampingLaw);

  arr posTrajectory = generateBrezel();

  mlr::Spline* spline = new mlr::Spline(posTrajectory.d0, posTrajectory);
  arr sp;
  uint n = 100;
  for(uint i = 0; i <= n; i++) {
    sp.append(spline->eval((double)i/n));
  }
  sp.reshape(n+1,3);
  plotLine(sp);


  TaskMap* posTask = new DefaultTaskMap(posTMT, *modelWorld, "endeffL");
  LinTaskSpaceAccLaw* posLaw = new LinTaskSpaceAccLaw(posTask, modelWorld, "endeffLPos");
  posLaw->setTrajectory(posTrajectory.d0, posTrajectory);
  posLaw->setC(eye(3)*1000.0);
  arr Kp = eye(3)*30.0;
  posLaw->setGains(Kp,eye(3)*5.0);

  laws.append(posLaw);



  controller->taskSpaceAccLaws = laws;
  controller->generateTaskSpaceSplines();
  pr2->executeTrajectory(10.0);

  modelWorld->watch(true, "press to stop");
  pr2->logState = false;
  pr2->logStateSave("test_2","tests");
  pr2->~PR2Interface();
}



int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  //followTrajectory();
  //qDotRefInConstraint();
  //qDotRefInConstraintAndSlide();
  //openSchublade();
  //testTorsoLiftLink();
  //openSchublade2();
  //testDemonstration();
  //openSchublade3();
  //controllerExample("nullSpace_2");
  //tischTouchdown_1();
  tests();
  return 0;
}
