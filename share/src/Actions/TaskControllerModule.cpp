#include "TaskControllerModule.h"
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>
#include <iostream> 
#include <fstream> 
using namespace std; 

#ifdef MLR_ROS
#  include <pr2/roscom.h>
#endif

TaskControllerModule::TaskControllerModule(const char* modelFile)
    : Module("TaskControllerModule", .01)
    , realWorld(modelFile?modelFile:mlr::mlrPath("data/pr2_model/pr2_model.ors").p)
    , feedbackController(NULL)
    , q0(realWorld.q)
    , oldfashioned(true)
    , useRos(false)
    , syncModelStateWithRos(false)
    , verbose(false)
    , useDynSim(true) {

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
  if(useRos) syncModelStateWithRos=true;

  if(useDynSim) {
    ors::KinematicWorld* dynWorld = new ors::KinematicWorld(realWorld); //TODO maybe change dynSim to accept reference
    dynSim->initializeSimulation(dynWorld);
    dynSim->startSimulation();
  }

}


void TaskControllerModule::step(){
  static uint t=0;
  t++;
  if(syncModelStateWithRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
  }

  ors::Joint *trans= realWorld.getJointByName("worldTranslationRotation");

  //-- read real state
  if(useRos){
    ctrl_obs.waitForNextRevision();
    pr2_odom.waitForRevisionGreaterThan(0);

    q_real = ctrl_obs.get()->q;
    qdot_real = ctrl_obs.get()->qdot;
    if(q_real.N==realWorld.q.N && qdot_real.N==realWorld.q.N){ //we received a good reading
      q_real.subRef(trans->qIndex, trans->qIndex+2) = pr2_odom.get();
      realWorld.setJointState(q_real, qdot_real);
      if(syncModelStateWithRos){
        q_model = q_real;
        qdot_model = qdot_real;
        modelWorld.set()->setJointState(q_model, qdot_model);
        cout <<"** GO!" <<endl;
        cout <<"REMOTE joint dimension=" <<q_real.N <<endl;
        cout <<"LOCAL  joint dimension=" <<realWorld.q.N <<endl;
        syncModelStateWithRos = false;
      }
    }else{
      if(t>20){
        HALT("sync'ing real PR2 with simulated failed")
      }
    }
  } else {
    if(useDynSim) {
      ctrl_obs.waitForNextRevision();
      q_real = ctrl_obs.get()->q;
      qdot_real = ctrl_obs.get()->qdot;
      realWorld.setJointState(q_real, qdot_real);
      q_model = q_real;
      qdot_model = qdot_real;
      modelWorld.set()->setJointState(q_model, qdot_model); //TODO don't know why this changes the green init robot as well
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
    refs.gamma = 1.;
    refs.Kp = ARR(1.);
    refs.Kd = ARR(1.);
    refs.Ki = ARR(0.2);
    refs.fL = zeros(6);
    refs.fR = zeros(6);
    refs.KiFT.clear();
    refs.J_ft_inv.clear();
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
        t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.KiFT, refs.J_ft_inv, realWorld);
      }
    }
    if(count==1) refs.Kp = .5;
    ctrlTasks.deAccess();

  }else{

    ctrlTasks.readAccess();
    modelWorld.writeAccess();
    feedbackController->tasks = ctrlTasks();

    //if there are no tasks, just stabilize
    if(feedbackController->tasks.N < 1) {
      TaskMap* qItselfTask = new TaskMap_qItself();
      CtrlTask* qItselfLaw = new CtrlTask(qItselfTask);
      qItselfLaw->prec = 10.0;
      qItselfLaw->setGains(10.0, 1.0); //TODO tune those gains
      qItselfLaw->setTarget(qItselfLaw->map.phi(modelWorld()), zeros(qItselfLaw->map.dim_phi(modelWorld())));
      feedbackController->tasks.append(qItselfLaw);
    }

    //TODO qItself task for joint space stability
    if(true) {
      TaskMap* qItselfJointSpaceStabilityTask = new TaskMap_qItself();
      CtrlTask* qItselfJSStabilityLaw = new CtrlTask(qItselfJointSpaceStabilityTask);
      qItselfJSStabilityLaw->prec = 10.0;
      qItselfJSStabilityLaw->setGains(0.0, 5.0); //TODO tune those gains
      qItselfJSStabilityLaw->setTarget(qItselfJSStabilityLaw->map.phi(modelWorld()), zeros(qItselfJSStabilityLaw->map.dim_phi(modelWorld())));
      feedbackController->tasks.append(qItselfJSStabilityLaw);
    }

    arr u0, Kp, Kd;
    feedbackController->calcOptimalControlProjected(Kp, Kd, u0); // TODO: what happens when changing the LAWs?

    arr K_ft, J_ft_inv, fRef;
    double gamma;
    feedbackController->calcForceControl(K_ft, J_ft_inv, fRef, gamma);

    if(!useDynSim) { //TODO what is, if useROS == true? the modelWorld should be updated from the rosMsg? Maybe then we don't need two worlds
      feedbackController->fwdSimulateControlLaw(Kp, Kd, u0);
    }


    modelWorld.deAccess();
    ctrlTasks.deAccess();

//    this->sendCommand(u0, Kp, Kd, K_ft, J_ft_inv, fRef, gamma);
    refs.q =  zeros(q_model.N);
    refs.qdot = zeros(q_model.N);
    refs.gamma = gamma;
    refs.Kp = Kp;
    refs.Kd = Kd;
    refs.Ki = ARR(0.);
    refs.fL = fRef;
    refs.fR = zeros(6);
    refs.KiFT = K_ft;
    refs.J_ft_inv = J_ft_inv;
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
  if(useRos || useDynSim){
    ctrl_ref.set() = refs;
  }
}

void TaskControllerModule::close(){
//  fil.close();
  delete feedbackController;
}
