#include "TaskControllerModule.h"
#include <Motion/pr2_heuristics.h>

TaskControllerModule *globalTaskControllerModule=NULL;
TaskControllerModule *taskControllerModule(){
  return globalTaskControllerModule;
}

TaskControllerModule::TaskControllerModule()
  : Module("TaskControllerModule"), realWorld("model.kvg"), __modelWorld__(realWorld),
    feedbackController(__modelWorld__, true), q0(__modelWorld__.q), useRos(false), syncModelStateWithRos(false), verbose(false){
  modelWorld.linkToVariable(new Variable<ors::KinematicWorld>(__modelWorld__));
  globalTaskControllerModule=this;
}

TaskControllerModule::~TaskControllerModule(){
}

void TaskControllerModule::open(){
  modelWorld.get()->getJointState(q_model, qdot_model);

  feedbackController.H_rate_diag = MT::getParameter<double>("Hrate", 1.)*pr2_reasonable_W(modelWorld.set()());
  feedbackController.qitselfPD.y_ref = q0;
  feedbackController.qitselfPD.setGains(.0,10.);

  MT::open(fil,"z.TaskControllerModule");

  useRos = MT::getParameter<bool>("useRos",false);
  if(useRos) syncModelStateWithRos=true;
}

void TaskControllerModule::step(){
  static uint t=0;
  t++;
  if(syncModelStateWithRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
  }

  //-- read real state
  if(useRos){
    ctrl_obs.waitForNextRevision();
    q_real = ctrl_obs.get()->q;
    qdot_real = ctrl_obs.get()->qdot;
    if(q_real.N==realWorld.q.N && qdot_real.N==realWorld.q.N){ //we received a good reading
      realWorld.setJointState(q_real, qdot_real);
      if(syncModelStateWithRos){
        q_model = q_real;
        qdot_model = qdot_real;
        modelWorld.set()->setJointState(q_model, qdot_model);
        cout <<"** GO!" <<endl;
        syncModelStateWithRos = false;
      }
    }else{
      if(t>20){
        HALT("sync'ing real PR2 with simulated failed - using useRos=false")
      }
    }
  }
  if(syncModelStateWithRos){
    cout <<"REMOTE joint dimension=" <<q_real.N <<endl;
    cout <<"LOCAL  joint dimension=" <<realWorld.q.N <<endl;
  }

  if(!(t%5)){
    __modelWorld__.watch(false, STRING("model world state t="<<(double)t/100.));
  }

  //-- do the logic of transitioning between actions, stopping/sequencing them, querying their state

  //-- code to output force signals
  if(true){
    ors::Shape *ftL_shape = realWorld.getShapeByName("endeffForceL");
    arr fLobs = ctrl_obs.get()->fL;
    arr uobs =  ctrl_obs.get()->u_bias;
    if(fLobs.N && uobs.N){
      arr Jft, J;
      realWorld.kinematicsPos(NoArr,J,ftL_shape->body,&ftL_shape->rel.pos);
      realWorld.kinematicsPos_wrtFrame(NoArr,Jft,ftL_shape->body,&ftL_shape->rel.pos,realWorld.getShapeByName("l_ft_sensor"));
      Jft = inverse_SymPosDef(Jft*~Jft)*Jft;
      J = inverse_SymPosDef(J*~J)*J;
//      MT::arrayBrackets="  ";
      fil <<t <<' ' <<zeros(3) <<' ' << Jft*fLobs << " " << J*uobs << endl;
//      MT::arrayBrackets="[]";
    }
  }

  //-- copy the task to the local controller
  ctrlTasks.readAccess();
  modelWorld.writeAccess();
  feedbackController.tasks = ctrlTasks();

  //-- compute the feedback controller step and iterate to compute a forward reference
  //now operational space control
  for(uint tt=0;tt<10;tt++){
    arr a = feedbackController.operationalSpaceControl();
    q_model += .001*qdot_model;
    qdot_model += .001*a;
    feedbackController.setState(q_model, qdot_model);
  }
  if(verbose) feedbackController.reportCurrentState();
  modelWorld.deAccess();
  ctrlTasks.deAccess();

  //-- first zero references
  CtrlMsg refs;
  refs.q =  q_model;
  refs.qdot = zeros(q_model.N);
  refs.gamma = 1.;
  refs.Kp = ARR(1.);
  refs.Kd = ARR(1.);
  refs.fL = zeros(6);
  refs.fR = zeros(6);
  refs.Ki.clear();
  refs.J_ft_inv.clear();
  refs.u_bias = zeros(q_model.N);

  //-- compute the force feedback control coefficients
  uint count=0;
  ctrlTasks.readAccess();
  feedbackController.tasks = ctrlTasks();
  for(CtrlTask *t : feedbackController.tasks) {
    if(t->active && t->f_ref.N){
      count++;
      if(count!=1) HALT("you have multiple active force control tasks - NIY");
      t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.Ki, refs.J_ft_inv, realWorld);
    }
  }
  if(count==1) refs.Kp = .5;
  ctrlTasks.deAccess();

  //-- send the computed movement to the robot
  ctrl_ref.set() = refs;
}

void TaskControllerModule::close(){
  fil.close();
}
