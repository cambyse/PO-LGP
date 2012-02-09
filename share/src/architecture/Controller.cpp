#include "motion.h"
#include <MT/soc.h>
#include <MT/soc_inverseKinematics.h>

struct sMotionControllerProcess{
  soc::SocSystem_Ors sys;

  //INPUT
  Lock taskLock; //lock this if you change the task!!
  /*
  bool useBwdMsg, forceColLimTVs, fixFingers;
  arr bwdMsg_v, bwdMsg_Vinv; //optional: backward messages from a planner
  double maxJointStep; //computeMotionFromTaskVariables will generate a null-step if this limit is exceeded
  arr skinState;
  
  //OUTPUT
  arr q_reference, v_reference; //, q_orsInit;  //the SIMULATION state (the modules buffer double states, simulation is synchronized with modules in the loop)
  arr q_home; //posture as loaded from the ors file
  */
};

myController::myController():Process("MotionController"){
  s = new sMotionControllerProcess();
  controllerTask=NULL;
  motionPlan=NULL;
  hardwareReference=NULL;
  geometricState=NULL;
  skinPressure=NULL;

}

myController::~myController(){
  delete s;
}

void myController::open(){ MT_MSG("NIY") }

void myController::close(){ MT_MSG("NIY") }

void myController::step(){
  CHECK(controllerTask, "please set controllerMode before launching MotionPrimitive");
  CHECK(motionPlan, "please set motionPlan before launching MotionPrimitive");
  CHECK(hardwareReference, "please set controllerReference before launching MotionPrimitive");
  CHECK(geometricState, "please set geometricState before launching MotionPrimitive");
  CHECK(skinPressure, "please set skinPressure before launching MotionPrimitive");
  
  MT::wait(1.);  return;

  ControllerTask::ControllerMode mode=controllerTask->get_mode(this);

  if(mode==ControllerTask::noType){
    //stop -> don't change q_reference
    hardwareReference->set_v_reference(zeros(14,1), this);
    return;
  }

  if(mode==ControllerTask::followTrajectory){
    //-- check if converged
    if(motionPlan->get_converged(this)==false){
      //stop
      MT_MSG("trying to follow non-converged trajectory");
      hardwareReference->set_v_reference(zeros(14,1), this);
      return;
    }

    //-- first compute the interpolated
    double relTime = controllerTask->get_relativeRealTimeOfController(this);
    double timeScale = controllerTask->get_followTrajectoryTimeScale(this);
    arr q_plan = motionPlan->get_q_plan(this);
    double plan_tau = motionPlan->get_tau(this);

    //where to interpolate
    relTime += timeScale * 0.01; //!!! hard coded 10msec as basic control cycle
    uint timeStep= relTime/plan_tau;
    double inter = relTime/plan_tau - (double)timeStep;  //same as fmod

    //do interpolation
    arr q_reference;
    if(timeStep+1<q_plan.d0){ //ok, time step is within the plan
      q_reference = (1.-inter)*q_plan[timeStep] + inter*q_plan[timeStep+1];
    }else{ //time step is beyond the horizon of the plan
      q_reference = q_plan[q_plan.d0-1];
      controllerTask->set_mode(ControllerTask::done, this);
    }

    //-- now test for collision
    MT_MSG("TODO");

    //-- pass to MotionReference
    hardwareReference->set_q_reference(q_reference, this);
  }
  
  if(mode==ControllerTask::feedback){
    bool forceColLimTVs = controllerTask->get_forceColLimTVs(this);
    bool fixFingers = controllerTask->get_fixFingers(this);

    skinPressure->readAccess(this);
    arr skinState = skinPressure->y_real;
    skinPressure->deAccess(this);

    //update the controllers own internal ors state - pulling from MotionReference
    arr q_reference = hardwareReference->get_q_reference(this);
    arr v_reference = hardwareReference->get_v_reference(this);
    s->sys.setqv(q_reference, v_reference);
    MT_MSG("solle das socSystem nicht die task variablen liste enthalten? wird doch geupdated?");

    //pull for possible changes in the geometric state
    MT_MSG("TODO");

    //update all task variables using this ors state
    TaskAbstraction *task = controllerTask->get_feedbackControlTask(this);
    s->taskLock.writeLock();
    task->updateTaskGoals(NULL);

    //=== compute motion from the task variables
    //check if a collition and limit variable are active
    bool colActive=false, limActive=false;
    uint i; TaskVariable *v;
    for_list(i, v, s->sys.vars) if(v->active){
      //?? ist sys.vars und task->vars eigentlich das gleiche??
      if(v->type==collTVT) colActive=true;
      if(v->type==qLimitsTVT) limActive=true;
    }
    if(forceColLimTVs && (!colActive || !limActive)) HALT("SAFETY BREACH! You need an active collision and limit variable!");

    //compute the motion step
    arr q_old=q_reference;
    arr dq, x, x_1;
  
    //dynamic control using SOC
    x_1=q_reference; x_1.append(v_reference);
    soc::bayesianDynamicControl(s->sys, x, x_1, 0);
    q_reference = x.sub(0, q_reference.N-1);
    v_reference = x.sub(v_reference.N, -1);

    //perhaps fix fingers
    if(fixFingers) for(uint j=7; j<14; j++){ v_reference(j)=0.; q_reference(j)=q_old(j); }
    s->taskLock.unlock();

    //SAFTY CHECK: too large steps?
    double step=euclideanDistance(q_reference, q_old);
    if(step>maxJointStep){
      MT_MSG(" *** WARNING *** too large step -> step |dq|=" <<step);
      q_reference=q_old + (q_reference-q_old)*maxJointStep/step;
      v_reference *= .5*maxJointStep/step;
      step=euclideanDistance(q_reference, q_old);
      MT_MSG(" *** WARNING *** too large step -> scaling to |dq_new|=" <<step);
      //v_reference.setZero(); SD: making too large step warnig  use max allowed step
    }
  
    hardwareReference->set_q_reference(q_reference, this);
    hardwareReference->set_v_reference(v_reference, this);

    //push proxies to the geometric state
    MT_MSG("TODO");
    /* Eigentlich spield controller iM eine double role: als
       q_reference berechnen, und die kinematic/proxies/taskvariables
       mit ors berechnen -> 2 Prozesse?
     */
    /*if(proxiesVar){
      proxiesVar->writeAccess(this);
      listCopy(proxiesVar->proxies, ors.proxies);
      proxiesVar->deAccess(this);
      } else MT_MSG("Variable pointer not set");*/
  }

  if(mode==ControllerTask::done){
    return;
  }

}
