#include "TaskControllerModule.h"
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>
#include <iostream> 
#include <fstream> 
using namespace std; 

#ifdef MLR_ROS
#  include <pr2/roscom.h>
#endif

void lowPassUpdate(arr& lowPass, const arr& signal, double rate=.01){
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

  oldfashioned = mlr::getParameter<bool>("oldfashinedTaskControl", true);
  useDynSim = mlr::getParameter<bool>("useDynSim", true);
}

TaskControllerModule::~TaskControllerModule(){
}

void changeColor(void*){  orsDrawColors=false; glColor(.8, 1., .8, .5); }
void changeColor2(void*){  orsDrawColors=true; orsDrawAlpha=1.; }

void TaskControllerModule::open(){
  modelWorld.set() = realWorld;
  feedbackController = new FeedbackMotionControl(modelWorld.set()(), true);

  modelWorld.get()->getJointState(q_model, qdot_model);

  feedbackController->H_rate_diag = mlr::getParameter<double>("Hrate", 1.)*pr2_reasonable_W(modelWorld.set()());
  feedbackController->qitselfPD.y_ref = q0;
  feedbackController->qitselfPD.setGains(0., 10.);

  //  mlr::open(fil,"z.TaskControllerModule");

#if 1
  modelWorld.writeAccess();
  modelWorld().gl().add(changeColor);
  modelWorld().gl().add(ors::glDrawGraph, &realWorld);
  modelWorld().gl().add(changeColor2);
  modelWorld.deAccess();
#endif


  useRos = mlr::getParameter<bool>("useRos",false);
  if(useRos || !oldfashioned) syncModelStateWithReal=true;

  if(!oldfashioned && !useRos) {
    dynSim = new RTControllerSimulation();
    dynSim->threadLoop();
  }

}


void TaskControllerModule::step(){
  static uint t=0;
  t++;

  ors::Joint *trans= realWorld.getJointByName("worldTranslationRotation");

  //-- read real state
  if(useRos || !oldfashioned){
    ctrl_obs.waitForNextRevision();
    if(useRos) pr2_odom.waitForRevisionGreaterThan(0);

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

//        if(q_history.d0>0) lowPassUpdate(q_lowPass, q_history[0]);
//        if(q_history.d0>1) lowPassUpdate(qdot_lowPass, (q_history[0]-q_history[1])/.01);
//        if(q_history.d0>2) lowPassUpdate(qddot_lowPass, (q_history[0]-2.*q_history[1]+q_history[2])/(.01*.01));
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

  //-- code to output force signals
  if(false){
    ors::Shape *ftL_shape = realWorld.getShapeByName("endeffForceL");
    arr fLobs = ctrl_obs.get()->fL;
    arr uobs =  ctrl_obs.get()->u_bias;
    if(fLobs.N && uobs.N){
      arr Jft, J;
      realWorld.kinematicsPos(NoArr, J, ftL_shape->body, ftL_shape->rel.pos);
      realWorld.kinematicsPos_wrtFrame(NoArr, Jft, ftL_shape->body, ftL_shape->rel.pos, realWorld.getShapeByName("l_ft_sensor"));
      Jft = inverse_SymPosDef(Jft*~Jft)*Jft;
      J = inverse_SymPosDef(J*~J)*J;
      //      mlr::arrayBrackets="  ";
      //      fil <<t <<' ' <<zeros(3) <<' ' <<Jft*fLobs << " " <<J*uobs << endl;
      //      mlr::arrayBrackets="[]";
    }
  }

  //-- copy the tasks to the local controller

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

    //count active tasks:
    uint ntasks=0;
    for(CtrlTask *t:feedbackController->tasks) if(t->active) ntasks++;

    //if there are no tasks, just stabilize
    if(false || !ntasks) { //is done in feedbackController->qitself
      if(!noTaskTask) noTaskTask = new CtrlTask("noTaskTask", new TaskMap_qItself());
      noTaskTask->prec = 10.0;
      noTaskTask->setGains(.0, 1.0); //TODO tune those gains
//      noTaskTask->setTarget(modelWorld().q);
      noTaskTask->active = true;
      feedbackController->tasks.append(noTaskTask);
    }

    //TODO qItself task for joint space stability
    if(false) {
      TaskMap* qItselfJointSpaceStabilityTask = new TaskMap_qItself();
      CtrlTask* qItselfJSStabilityLaw = new CtrlTask("qItselfJSStabilityLaw", qItselfJointSpaceStabilityTask);
      qItselfJSStabilityLaw->prec = 10.0;
      qItselfJSStabilityLaw->setGains(0.0, 5.0); //TODO tune those gains
      qItselfJSStabilityLaw->setTarget(qItselfJSStabilityLaw->map.phi(modelWorld()), zeros(qItselfJSStabilityLaw->map.dim_phi(modelWorld())));
      feedbackController->tasks.append(qItselfJSStabilityLaw);
    }

#if 0
    arr u0, Kp, Kd;
    arr M, F;
    feedbackController->world.equationOfMotion(M, F, false);
//    if(model_error_g.N) FplusG += 0.1 * model_error_g;
    arr u_mean = feedbackController->calcOptimalControlProjected(Kp, Kd, u0, M, F);
//    lowPassUpdate(u_lowPass, u_mean);

//    if(qddot_lowPass.N){
//      model_error_g = u_lowPass - M*qddot_lowPass - F;
//      cout <<"model error = " <<sumOfSqr(model_error_g) <<endl;
//    }
#else
    arr a, Kp, Kd, k;
    a = feedbackController->getDesiredLinAccLaw(Kp, Kd, k);
    arr a0 = feedbackController->operationalSpaceControl();
    cout <<" a=" <<a <<endl <<"a0=" <<a0 <<endl;

    arr M, F;
    feedbackController->world.equationOfMotion(M, F, false);
    arr u0 = M*k + F;
    Kp = M*Kp;
    Kd = M*Kd;
#endif

    arr K_ft, J_ft_inv, fRef;
    double gamma;
    feedbackController->calcForceControl(K_ft, J_ft_inv, fRef, gamma);

    //    if(!useDynSim) { //TODO what is, if useROS == true? the modelWorld should be updated from the rosMsg? Maybe then we don't need two worlds
    //      feedbackController->fwdSimulateControlLaw(Kp, Kd, u0);
    //    }

    if(verbose){
      LOG(0) <<"************** Tasks Report **********";
      feedbackController->reportCurrentState();
    }
    modelWorld.deAccess();
    ctrlTasks.deAccess();

    //    this->sendCommand(u0, Kp, Kd, K_ft, J_ft_inv, fRef, gamma);
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
    refs.u_bias = u0;
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
  //  fil.close();
  delete feedbackController;
}
