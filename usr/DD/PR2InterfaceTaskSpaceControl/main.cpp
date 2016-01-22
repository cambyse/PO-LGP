#include <Core/array.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/motion.h>
#include <pr2/roscom.h>
#include <Core/util.h>
#include <Gui/plot.h>

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

  TaskMap* gazeAtMap = new DefaultTaskMap(gazeAtTMT, *modelWorld, "endeffKinect", NoVector, "endeffL");
  LinTaskSpaceAccLaw* gazeAtLaw = new LinTaskSpaceAccLaw(gazeAtMap, modelWorld);
  gazeAtLaw->setC(eye(2)*1000.0);
  gazeAtLaw->setGains(eye(2)*10.0,eye(2)*5.0);
  gazeAtLaw->setTrajectory(3, zeros(3,2));

  controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(posLaw);
  controller->addLinTaskSpaceAccLaw(orientationLaw);
  controller->addLinTaskSpaceAccLaw(qDampingLaw);
  controller->addLinTaskSpaceAccLaw(limitsLaw);
  controller->addLinTaskSpaceAccLaw(velLaw);
  //controller->addLinTaskSpaceAccLaw(gazeAtLaw);

  controller->generateTaskSpaceSplines();

  pr2->executeTrajectory(10.0);
  mlr::wait(0.5);

  arr slideTraj;
  slideTraj.append(~ARR(0.7,0.0,0.55));
  slideTraj.append(~ARR(0.7,0.15,0.55));
  slideTraj.append(~ARR(0.7,0.3,0.55));

  posLaw->setTrajectory(slideTraj.d0, slideTraj);

  controller->generateTaskSpaceSplines();
  pr2->executeTrajectory(10.0);


  pr2->logState = false;
  pr2->logStateSave("touchdownAndSlide_3");
  modelWorld->watch(true, "Press to stop");

  pr2->~PR2Interface();
}


int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  //followTrajectory();
  //qDotRefInConstraint();
  qDotRefInConstraintAndSlide();
  return 0;
}
