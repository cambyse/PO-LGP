#include "TaskControllerModule.h"
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>
#include <iostream> 
#include <fstream> 
using namespace std; 

#ifdef MLR_ROS
#  include <pr2/roscom.h>
#endif

void lowPassUpdate(arr& lowPass, const arr& signal, double rate=.1){
  if(lowPass.N!=signal.N){ lowPass=zeros(signal.N); return; }
  lowPass = (1.-rate)*lowPass + rate*signal;
}

TaskControllerModule::TaskControllerModule(const char* modelFile)
  : Module("TaskControllerModule", .01)
  , realWorld(modelFile?modelFile:mlr::mlrPath("data/pr2_model/pr2_model.ors").p)
  , feedbackController(NULL)
  , q0(realWorld.q)
  , oldfashioned(true)
  , useRos(false)
  , isInSyncWithRobot(false)
  , syncModelStateWithReal(false)
  , verbose(false)
  , useDynSim(true)
  , noTaskTask(NULL){

  useRos = mlr::getParameter<bool>("useRos",false);
  oldfashioned = mlr::getParameter<bool>("oldfashinedTaskControl", true);
  useDynSim = !oldfashioned && !useRos; //mlr::getParameter<bool>("useDynSim", true);
}

TaskControllerModule::~TaskControllerModule(){
}

void changeColor(void*){  orsDrawColors=false; glColor(.8, 1., .8, .5); }
void changeColor2(void*){  orsDrawColors=true; orsDrawAlpha=1.; }

void TaskControllerModule::open(){
  modelWorld.set() = realWorld;
  feedbackController = new FeedbackMotionControl(modelWorld.set()(), true);

  modelWorld.get()->getJointState(q_model, qdot_model);

  feedbackController->qNullCostRef.y_ref = q0;
  feedbackController->qNullCostRef.setGains(0., 10.);
  feedbackController->qNullCostRef.prec = mlr::getParameter<double>("Hrate", 1.)*pr2_reasonable_W(modelWorld.set()());

#if 1
  modelWorld.writeAccess();
  modelWorld().gl().add(changeColor);
  modelWorld().gl().add(ors::glDrawGraph, &realWorld);
  modelWorld().gl().add(changeColor2);
  modelWorld.deAccess();
#endif

  if(useRos || !oldfashioned) syncModelStateWithReal=true;

  if(!oldfashioned && !useRos) {
    dynSim = new RTControllerSimulation(0.01, false, 1.);
    dynSim->threadLoop();
  }

  dataFiles.open({"q", "qdot", "qddot", "q_low", "qdot_low", "qddot_low", "qddot_des"});
}


void TaskControllerModule::step(){
  static uint t=0;
  t++;

  ors::Joint *trans= realWorld.getJointByName("worldTranslationRotation");

  //-- read real state
  if(useRos || !oldfashioned){
    ctrl_obs.waitForNextRevision();
    if(useRos) pr2_odom.waitForRevisionGreaterThan(0);

    qdot_last = qdot_real;
    q_real = ctrl_obs.get()->q;
    qdot_real = ctrl_obs.get()->qdot;
    if(q_real.N==realWorld.q.N && qdot_real.N==realWorld.q.N){ //we received a good reading
      if(useRos) q_real.subRef(trans->qIndex, trans->qIndex+2) = pr2_odom.get();
      realWorld.setJointState(q_real, qdot_real);
      if(syncModelStateWithReal){
        q_model = q_real;
        qdot_model = qdot_real;
        modelWorld.set()->setJointState(q_model, qdot_model);
        q_history.prepend(q_real); q_history.reshape(q_history.N/q_real.N, q_real.N);
        if(q_history.d0>5) q_history.resizeCopy(5, q_real.N);

        if(q_history.d0>0) lowPassUpdate(q_lowPass, q_history[0]);
        if(q_history.d0>1) lowPassUpdate(qdot_lowPass, (q_history[0]-q_history[1])/.01);
        if(q_history.d0>2) lowPassUpdate(qddot_lowPass, (q_history[0]-2.*q_history[1]+q_history[2])/(.01*.01));
      }
      if(oldfashioned) syncModelStateWithReal = false;
      isInSyncWithRobot = true;
    }else{
      cout <<"** Waiting for ROS message on initial configuration.." <<endl;
      if(t>20){
        HALT("sync'ing real PR2 with simulated failed")
      }
    }
  }

  //-- sync the model world with the AlvarMarkers
  modelWorld.writeAccess();
  AlvarMarkers alvarMarkers = ar_pose_marker.get();
  syncMarkers(modelWorld(), alvarMarkers);
  syncMarkers(realWorld, alvarMarkers);
  modelWorld.deAccess();

  //-- display the model world (and in same gl, also the real world)
  if(!(t%5)){
#if 1
    modelWorld.set()->watch(false, STRING("model world state t="<<(double)t/100.));
#endif
  }

  //-- compute the feedback controller step and iterate to compute a forward reference
  CtrlMsg refs;
  if(oldfashioned){

    //now operational space control
    ctrlTasks.readAccess();
    modelWorld.writeAccess();
    feedbackController->tasks = ctrlTasks();
    for(uint tt=0;tt<10;tt++){
      arr a = feedbackController->operationalSpaceControl();
      q_model += .001*qdot_model;
      qdot_model += .001*a;
      if(fixBase.get()) {
        qdot_model(trans->qIndex+0) = 0;
        qdot_model(trans->qIndex+1) = 0;
        qdot_model(trans->qIndex+2) = 0;
        //      q_model(trans->qIndex+0) = 0;
        //      q_model(trans->qIndex+1) = 0;
        //      q_model(trans->qIndex+2) = 0;
      }
      feedbackController->setState(q_model, qdot_model);
    }
    if(verbose) feedbackController->reportCurrentState();
    modelWorld.deAccess();
    ctrlTasks.deAccess();

    //-- first zero references
    refs.q =  q_model;
    refs.qdot = zeros(q_model.N);
    refs.fL_gamma = 1.;
    refs.Kp = ARR(1.);
    refs.Kd = ARR(1.);
    refs.Ki = ARR(0.2);
    refs.fL = zeros(6);
    refs.fR = zeros(6);
    refs.KiFTL.clear();
    refs.J_ft_invL.clear();
    refs.u_bias = zeros(q_model.N);
    refs.intLimitRatio = 0.7;

    //-- compute the force feedback control coefficients
    uint count=0;
    ctrlTasks.readAccess();
    feedbackController->tasks = ctrlTasks();
    for(CtrlTask *t : feedbackController->tasks) {
      if(t->active && t->f_ref.N){
        count++;
        if(count!=1) HALT("you have multiple active force control tasks - NIY");
        t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.KiFTL, refs.J_ft_invL, realWorld);
      }
    }
    if(count==1) refs.Kp = .5;
    ctrlTasks.deAccess();

  }else{

    ctrlTasks.readAccess();
    modelWorld.writeAccess();
    feedbackController->tasks = ctrlTasks();

//    //count active tasks:
//    uint ntasks=0;
//    for(CtrlTask *t:feedbackController->tasks) if(t->active) ntasks++;

//    //if there are no tasks, just stabilize
//    if(true || !ntasks) { //is done in feedbackController->qitself
//      if(!noTaskTask) noTaskTask = new CtrlTask("noTaskTask", new TaskMap_qItself());
//      noTaskTask->prec = 10.0;
//      noTaskTask->setGains(.0, 1.0); //TODO tune those gains
//      //      noTaskTask->setTarget(modelWorld().q);
//      noTaskTask->active = true;
//      feedbackController->tasks.append(noTaskTask);
//    }

//    //TODO qItself task for joint space stability
//    if(false) {
//      TaskMap* qItselfJointSpaceStabilityTask = new TaskMap_qItself();
//      CtrlTask* qItselfJSStabilityLaw = new CtrlTask("qItselfJSStabilityLaw", qItselfJointSpaceStabilityTask);
//      qItselfJSStabilityLaw->prec = 10.0;
//      qItselfJSStabilityLaw->setGains(0.0, 5.0); //TODO tune those gains
//      qItselfJSStabilityLaw->setTarget(qItselfJSStabilityLaw->map.phi(modelWorld()), zeros(qItselfJSStabilityLaw->map.dim_phi(modelWorld())));
//      feedbackController->tasks.append(qItselfJSStabilityLaw);
//    }

#if 0

    arr u_bias, Kp, Kd;
    arr M, F;
    feedbackController->world.equationOfMotion(M, F, false);
    arr u_mean = feedbackController->calcOptimalControlProjected(Kp, Kd, u_bias, M, F); // TODO: what happens when changing the LAWs?

#else

    //compute desired acceleration law in q-space
    arr a, Kp, Kd, k;
    a = feedbackController->getDesiredLinAccLaw(Kp, Kd, k);

    //translate to motor torques
    arr M, F;
    feedbackController->world.equationOfMotion(M, F, false);
    arr u_bias = M*k + F;
    Kp = M*Kp;
    Kd = M*Kd;

    checkNan(u_bias);

    //-- compute the error between expected change in velocity and true one
    if(!a_last.N) a_last = a;
    if(!qdot_last.N) qdot_last = qdot_real;
    arr a_err = (qdot_real - qdot_last)/.01 - a_last;
    a_last = a;
    // integrate this error
    if(!aErrorIntegral.N) aErrorIntegral = a_err;
    else aErrorIntegral += a_err;
    // add integral error to control bias
    u_bias -= .01 * M * aErrorIntegral;

#endif

    // F/T limit control
    arr K_ft, J_ft_inv, fRef;
    double gamma;
    feedbackController->calcForceControl(K_ft, J_ft_inv, fRef, gamma);


    if(verbose){
      LOG(0) <<"************** Tasks Report **********";
      feedbackController->reportCurrentState();
    }

//    dataFiles.write({&modelWorld().q, &modelWorld().qdot, &qddot, &q_lowPass, &qdot_lowPass, &qddot_lowPass, &aErrorIntegral});

    modelWorld.deAccess();
    ctrlTasks.deAccess();

    refs.q =  zeros(q_model.N);
    refs.qdot = zeros(q_model.N);
    refs.fL_gamma = gamma;
    refs.Kp = Kp;
    refs.Kd = Kd;
    refs.Ki = ARR(0.);
    refs.fL = fRef;
    refs.fR = zeros(6);
    refs.KiFTL = K_ft;
    refs.J_ft_invL = J_ft_inv;
    refs.u_bias = u_bias;
    refs.intLimitRatio = 0.7;
  }

  //-- send base motion command
  if(useRos) {
    if (!fixBase.get() && trans && trans->qDim()==3) {
      refs.qdot(trans->qIndex+0) = qdot_model(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot_model(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot_model(trans->qIndex+2);
    }
  }

  //-- send the computed movement to the robot
  if(isInSyncWithRobot && (useRos || useDynSim)){
    ctrl_ref.set() = refs;
  }
}

void TaskControllerModule::close(){
  delete feedbackController;
}
