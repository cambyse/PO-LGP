#include "pr2Interface.h"

#include <stdlib.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <Core/module.h>
#include <Gui/opengl.h>
#include <Core/util.h>

PR2Interface::PR2Interface() : Module("PR2_Interface") {
  this->useROS = mlr::getParameter<bool>("useRos", false);
}

void PR2Interface::step() {
  arr qRealWorld, qDotRealWorld;

  this->ctrl_obs.waitForNextRevision();
  CtrlMsg actMsg = ctrl_obs.get();
  qRealWorld = actMsg.q;
  qDotRealWorld = actMsg.qdot;

  this->realWorld->setJointState(qRealWorld, qDotRealWorld);
  this->realWorld->watch(false);

  arr qModelWorld, qDotModelWorld;

  transferQbetweenTwoWorlds(qModelWorld, qRealWorld, *this->modelWorld, *this->realWorld);
  transferQDotbetweenTwoWorlds(qDotModelWorld, qDotRealWorld, *this->modelWorld, *this->realWorld);

  this->modelWorld->setJointState(qModelWorld, qDotModelWorld);
  this->modelWorld->watch(false);

  //TODO if there is no law, maybe a jointSpace law should be there with low gains?
  arr u0, Kp, Kd;
  this->controller->calcOptimalControlProjected(Kp,Kd,u0); // TODO: what happens when changing the LAWs?
  this->sendCommand(u0, Kp, Kd);

  cout << actMsg.fL(2) << endl;

  if(this->logState) {
    this->logQObs.append(~qRealWorld);
    this->logQDotObs.append(~qDotRealWorld);
    this->logFLObs.append(~actMsg.fL);
    this->logFRObs.append(~actMsg.fR);
    for(LinTaskSpaceAccLaw* law : this->controller->taskSpaceAccLaws) {
      arr y, J, yDot, q, qDot;
      law->world->getJointState(q, qDot);
      law->getPhi(y, J);
      yDot = J*qDot;
      this->logMap[STRING(law->name << "Obs")].append(~y);
      this->logMap[STRING(law->name << "Ref")].append(~law->getRef());
      this->logMap[STRING(law->name << "DotObs")].append(~yDot);
      this->logMap[STRING(law->name << "DotRef")].append(~law->getDotRef());
    }
  }
}

void PR2Interface::initialize(ors::KinematicWorld* realWorld, ors::KinematicWorld* realWorldSimulation, ors::KinematicWorld* modelWorld, TaskSpaceController* controller) {

  cout << "TODO: nochmal eine World mehr" << endl;

  this->controller = controller;

  this->modelWorld = modelWorld;
  this->modelWorld->meldFixedJoints();
  this->modelWorld->gl().title = "Model World";
  this->modelWorld->watch(false);

  if(this->useROS) {
    this->realWorld = realWorld;
    this->realWorld->gl().title = "Real World";
    cout << "Trying to connect to PR2" << endl;

    rosCheckInit();
    new RosCom_Spinner();
    new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
    new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
    threadOpenModules(true);

    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    while(true) {
      this->ctrl_obs.var->waitForNextRevision(); // TODO why .var???
      cout << "REMOTE joint dimension = " << this->ctrl_obs.get()->q.N << endl;
      cout << "LOCAL  joint dimension = " << this->realWorld->q.N << endl;

      if(this->ctrl_obs.get()->q.N == this->realWorld->q.N && this->ctrl_obs.get()->qdot.N == this->realWorld->q.N) {
        cout << "Syncing successfull :-)" << endl;
        break;
      }
    }

    //TODO initMsg should contain gains?
    CtrlMsg initMsg;
    initMsg.fL = zeros(6);
    initMsg.KiFT.clear();
    initMsg.J_ft_inv.clear();
    initMsg.u_bias = zeros(this->realWorld->getJointStateDimension());
    initMsg.Kp = ARR(0.0);
    initMsg.Kd = ARR(0.0);
    initMsg.Ki = ARR(0.0);
    initMsg.gamma = 1.;
    initMsg.velLimitRatio = .1;
    initMsg.effLimitRatio = 1.;
    initMsg.intLimitRatio = 0.8;

    initMsg.q = this->realWorld->getJointState();
    initMsg.qdot = zeros(this->realWorld->getJointStateDimension());

    this->ctrlMsg = initMsg;

    this->ctrl_ref.set() = ctrlMsg;

  } else {
    this->realWorld = realWorldSimulation;
    this->realWorld->gl().title = "Real World Simulated";
    this->dynamicSimulation = new DynamicSimulation();
    threadOpenModules(true);
    this->dynamicSimulation->initializeSimulation(new ors::KinematicWorld(*this->realWorld));
    this->dynamicSimulation->startSimulation();
  }
  this->realWorld->watch(false);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  cout << "%           init PR2 interface           %" << endl;
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
}

void PR2Interface::initialize(ors::KinematicWorld* realWorld, ors::KinematicWorld* modelWorld, TaskSpaceController* controller) {
  this->initialize(realWorld, realWorld, modelWorld, controller);
}

void PR2Interface::startInterface() {
  this->realWorld->watch(true, "Press Enter to start everything :-) :-) :-)");
  this->realWorld->watch(false, "");
  this->threadLoop();
  mlr::wait(1.0);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  cout << "%        start PR2 interface             %" << endl;
  cout << "%           and LOOOOOOP!                %" << endl;
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
}

void PR2Interface::sendCommand(arr u0, arr Kp, arr Kd) {
  arr u0RealWorld, KpRealWorld, KdRealWorld, qRefRealWorld, qDotRefRealWorld;
  transferU0BetweenTwoWorlds(u0RealWorld, u0, *this->realWorld, *this->modelWorld);
  transferKpBetweenTwoWorlds(KpRealWorld, Kp, *this->realWorld, *this->modelWorld);
  transferKdBetweenTwoWorlds(KdRealWorld, Kd, *this->realWorld, *this->modelWorld);
  transferQbetweenTwoWorlds(qRefRealWorld, zeros(modelWorld->getJointStateDimension()), *this->realWorld, *this->modelWorld);
  transferQDotbetweenTwoWorlds(qDotRefRealWorld, zeros(modelWorld->getJointStateDimension()), *this->realWorld, *this->modelWorld);

  this->ctrlMsg.u_bias = u0RealWorld;
  this->ctrlMsg.Kp = KpRealWorld;
  this->ctrlMsg.Kd = KdRealWorld;
  this->ctrlMsg.q = qRefRealWorld;
  this->ctrlMsg.qdot = qDotRefRealWorld;

  //this->ctrl_ref.writeAccess();
  this->ctrl_ref.set() = ctrlMsg;
  //this->ctrl_ref.deAccess();

  if(logState) {
    this->logU0.append(~u0RealWorld);
    this->logKp.append(~KpRealWorld);
    this->logKd.append(~KdRealWorld);
    this->logQRef.append(~qRefRealWorld);
    this->logQDotRef.append(~qDotRefRealWorld);
  }
}

void PR2Interface::goToPosition(arr pos, double executionTime) {
  NIY;
}

void PR2Interface::goToTasks(mlr::Array<LinTaskSpaceAccLaw*> laws, double executionTime, bool useMotionPlanner) {
  if(useMotionPlanner) {
    ors::KinematicWorld copiedWorld(*this->modelWorld);
    MotionProblem MP(copiedWorld);

    MP.x0 = modelWorld->getJointState();

    Task *t;
    t = MP.addTask("transitions", new TransitionTaskMap(MP.world));
    t->map.order=2; //make this an acceleration task!
    t->setCostSpecs(0, MP.T, {0.}, 1e0);

    t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
    t->setCostSpecs(0., MP.T, {0.}, 1.0);
    t = MP.addTask("qLimits", new LimitsConstraint());
    t->setCostSpecs(0., MP.T, {0.}, 1.);

    for(LinTaskSpaceAccLaw* law : laws) {
      t = MP.addTask(law->name, law->map);
      t->setCostSpecs(MP.T-2, MP.T, law->getRef(), 10.0);
    }

    MotionProblemFunction MF(MP);

    arr traj = MP.getInitialization();

    optConstrained(traj, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

    //MP.costReport();

    showTrajectory(traj, *this->modelWorld);

    TaskMap* qTask = new TaskMap_qItself();
    LinTaskSpaceAccLaw* qLaw = new LinTaskSpaceAccLaw(qTask, this->modelWorld, "qLaw");
    qLaw->setC(eye(this->modelWorld->getJointStateDimension())*1000.0);
    qLaw->setGains(eye(this->modelWorld->getJointStateDimension())*15.0, eye(this->modelWorld->getJointStateDimension())*5.0);
    qLaw->setTrajectory(traj.d0, traj);

    this->controller->taskSpaceAccLaws.clear();
    controller->addLinTaskSpaceAccLaw(qLaw);
    controller->generateTaskSpaceSplines();
    this->executeTrajectory(executionTime);
  } else {
    NIY;
  }
}

void PR2Interface::goToJointState(arr jointState, double executionTime) {
  NIY;
}

void PR2Interface::goToTask(TaskMap* map, arr ref, double executionTime) {
  ors::KinematicWorld copiedWorld(*this->modelWorld);
  MotionProblem MP(copiedWorld);

  MP.x0 = modelWorld->getJointState();

  cout <<  MP.tau << endl;

  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(MP.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("Map", map);
  t->setCostSpecs(MP.T-5, MP.T, ref, 10.0);

  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  t->setCostSpecs(0., MP.T, {0.}, 1.0);
  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  MotionProblemFunction MF(MP);

  arr traj = MP.getInitialization();

  optConstrained(traj, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

  //MP.costReport();

  TaskMap* qTask = new TaskMap_qItself();
  LinTaskSpaceAccLaw* qLaw = new LinTaskSpaceAccLaw(qTask, this->modelWorld);
  qLaw->setC(eye(this->modelWorld->getJointStateDimension())*1000.0);
  qLaw->setGains(eye(this->modelWorld->getJointStateDimension())*10.0, eye(this->modelWorld->getJointStateDimension())*5.0);
  qLaw->setTrajectory(traj.d0, traj);

  this->controller->taskSpaceAccLaws.clear();
  controller->addLinTaskSpaceAccLaw(qLaw);
  controller->generateTaskSpaceSplines();
  this->executeTrajectory(executionTime);
}

void PR2Interface::executeTrajectory(double executionTime) {
  cout << "start executing trajectory" << endl;
  mlr::timerStart(true);
  double time = 0.0;
  uint n = 0;
  while(true) {
    double s;
    if(time < executionTime) {
      s = time/executionTime;
    } else {
      s = 1;
      cout << "finished execution of trajectory" << endl;
      //logStateSave();
      break;
    }

    for(LinTaskSpaceAccLaw* law : this->controller->taskSpaceAccLaws) {
      law->setTargetEvalSpline(s);
      /*if(this->logState) {
        arr y, J, yDot, q, qDot;
        law->world->getJointState(q, qDot);
        law->getPhi(y, J);
        yDot = J*qDot;
        this->logMap[STRING(law->name << "Obs")].append(~y);
        this->logMap[STRING(law->name << "Ref")].append(~law->getRef());
        this->logMap[STRING(law->name << "DotObs")].append(~yDot);
        this->logMap[STRING(law->name << "DotRef")].append(~law->getDotRef());
      }*/
    }

    /*arr Kp,Kd,u0;
    // TODO: Don't do this here, do it in the loop
    this->ctrl_obs.waitForNextRevision(); // TODO: necesarry?
    this->controller->calcOptimalControlProjected(Kp,Kd,u0);
    this->sendCommand(u0, Kp, Kd);*/

    n++;
    time += mlr::timerRead(true);
  }
}

void PR2Interface::logStateSave(mlr::String name, mlr::String folder) {
  cout << "start saving log " << name << endl;
  if(this->logQObs.N) write(LIST<arr>(this->logQObs), STRING(folder << "qObs" << "_" << name << ".dat"));
  if(this->logQRef.N) write(LIST<arr>(this->logQRef), STRING(folder << "qRef" << "_" << name << ".dat"));
  if(this->logQDotObs.N)write(LIST<arr>(this->logQDotObs), STRING(folder << "qDotObs" << "_" << name << ".dat"));
  if(this->logQDotRef.N)write(LIST<arr>(this->logQDotRef), STRING(folder << "qDotRef" << "_" << name << ".dat"));
  if(this->logU0.N)write(LIST<arr>(this->logU0), STRING(folder << "u0" << "_" << name << ".dat"));
  if(this->logKp.N)write(LIST<arr>(this->logKp), STRING(folder << "Kp" << "_" << name << ".dat"));
  if(this->logKd.N)write(LIST<arr>(this->logKd), STRING(folder << "Kd" << "_" << name << ".dat"));
  if(this->logFLObs.N)write(LIST<arr>(this->logFLObs), STRING(folder << "FLObs" << "_" << name << ".dat"));
  if(this->logFRObs.N)write(LIST<arr>(this->logFRObs), STRING(folder << "FRObs" << "_" << name << ".dat"));

  for(auto m : this->logMap) {
    write(LIST<arr>(m.second), STRING(folder << m.first << "_" << name << ".dat"));
  }
  cout << "finished saving log " << name << endl;
}

void PR2Interface::clearLog() {
  this->logQObs.clear();
  this->logQRef.clear();
  this->logQDotObs.clear();
  this->logQDotRef.clear();
  this->logU0.clear();
  this->logKp.clear();
  this->logKd.clear();
  this->logFLObs.clear();
  this->logFRObs.clear();
  logMap.clear();
  cout << "cleared logging" << endl;
}

REGISTER_MODULE(PR2Interface)



void showTrajectory(const arr& traj, ors::KinematicWorld& _world, bool copyWorld, double delay, mlr::String text) {
  ors::KinematicWorld* world;
  if(copyWorld) {
    world = new ors::KinematicWorld(_world);
  } else {
    world = &_world;
  }

  world->watch(true, STRING("Show trajectory. " << text << " "));
  for(uint i = 0; i < traj.d0; i++) {
    world->setJointState(traj[i]);
    world->watch(false);
    mlr::wait(delay);
  }
  world->watch(true, "Finished. ");
  if(copyWorld) {
    delete world;
  }
}
