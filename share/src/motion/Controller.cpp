#include "motion.h"
#include "FeedbackControlTasks.h"

#include <MT/soc.h>
#include <MT/soc_inverseKinematics.h>
#include <hardware/hardware.h>

struct sController {
  ControllerTask *controllerTask;
  MotionPlan *motionPlan;
  HardwareReference *hardwareReference;
  GeometricState *geo;
  
  ors::Graph *ors;
  soc::SocSystem_Ors sys;
  
  double tau;
  double maxJointStep;
  
  /*
  bool useBwdMsg, forceColLimTVs, fixFingers;
  arr bwdMsg_v, bwdMsg_Vinv; //optional: backward messages from a planner
  double maxJointStep; //computeMotionFromTaskVariables will generate a null-step if this limit is exceeded
  
  //OUTPUT
  arr q_reference, v_reference; //, q_orsInit;  //the SIMULATION state (the modules buffer double states, simulation is synchronized with modules in the loop)
  arr q_home; //posture as loaded from the ors file
  */
};

Controller::Controller():Process("MotionController") {
  s = new sController();
  birosInfo.getVariable(controllerTask, "ControllerTask", this);
  birosInfo.getVariable(motionPlan, "MotionPlan", this);
  birosInfo.getVariable(hardwareReference, "HardwareReference", this);
  birosInfo.getVariable(geo, "GeometricState", this);
}

Controller::~Controller() {
  delete s;
}

void Controller::open() {
  CHECK(geo, "please set geometricState before launching MotionPrimitive");
  arr W = birosInfo.getParameter<arr>("Controller_W", this);
  s->tau = birosInfo.getParameter<double>("Controller_tau", this);
  s->maxJointStep = birosInfo.getParameter<double>("Controller_maxJointStep", this);
  
  //clone the geometric state
  geo->readAccess(this);
  s->ors = geo->ors.newClone();
  geo->deAccess(this);
  
  s->sys.initBasics(s->ors, NULL, NULL,
                    1, s->tau, true, &W);
                    
}

void Controller::close() { MT_MSG("NIY") }

void Controller::step() {
  CHECK(controllerTask, "please set controllerMode before launching MotionPrimitive");
  CHECK(hardwareReference, "please set controllerReference before launching MotionPrimitive");
  CHECK(geo, "please set geometricState before launching MotionPrimitive");
  
  ControllerTask::ControllerMode mode=controllerTask->get_mode(this);
  
  if (mode==ControllerTask::stop) {
    //stop -> don't change q_reference
    hardwareReference->set_v_reference(zeros(14,1), this);
    controllerTask->waitForConditionSignal(.01);
    return;
  }
  
  if (mode==ControllerTask::followPlan) {
    CHECK(motionPlan, "please set motionPlan before launching MotionPrimitive");
    
    //-- check if converged
    if (motionPlan->get_converged(this)==false) {
      //stop
      //MT_MSG("trying to follow non-converged trajectory");
      hardwareReference->set_v_reference(zeros(14,1), this);
      motionPlan->waitForConditionSignal(.01);
      return;
    }
    
    //-- first compute the interpolated
    double realTime = controllerTask->get_relativeRealTimeOfController(this);
    double timeScale = controllerTask->get_followTrajectoryTimeScale(this);
    arr q_plan = motionPlan->get_q_plan(this);
    double plan_tau = motionPlan->get_tau(this);
    
    //where to interpolate
    realTime += timeScale * s->tau; //!!! hard coded 10msec as basic control cycle
    uint timeStep= realTime/plan_tau;
    double inter = realTime/plan_tau - (double)timeStep;  //same as fmod
    
    //do interpolation
    arr q_reference;
    if (timeStep+1<q_plan.d0) { //ok, time step is within the plan
      q_reference = (1.-inter)*q_plan[timeStep] + inter*q_plan[timeStep+1];
    } else { //time step is beyond the horizon of the plan
      q_reference = q_plan[q_plan.d0-1];
    }
    
    //perhaps fix fingers
    bool fixFingers = controllerTask->get_fixFingers(this);
    if (fixFingers) for (uint j=7; j<14; j++) {
      q_reference(j)=q_plan[0](j); 
    }
    
    cout <<"Following trajectory: realTime=" <<realTime <<" step=" <<timeStep <<'+' <<inter <<endl;
    controllerTask->set_relativeRealTimeOfController(realTime, this);
    
    if (timeStep>=q_plan.d0-1) {
      controllerTask->set_mode(ControllerTask::done, this);
      controllerTask->set_relativeRealTimeOfController(0., this);
    }
    
    //-- now test for collision
    //MT_MSG("TODO");
    
    //-- pass to MotionReference
    hardwareReference->set_q_reference(q_reference, this);
  }
  
  if (mode==ControllerTask::feedback) {
    bool forceColLimTVs = controllerTask->get_forceColLimTVs(this);
    bool fixFingers = controllerTask->get_fixFingers(this);
    
    //pull for possible changes in the geometric state
    //MT_MSG("TODO");
    
    //update the controllers own internal ors state - pulling from MotionReference
    arr q_reference = hardwareReference->get_q_reference(this);
    arr v_reference = hardwareReference->get_v_reference(this);
    s->sys.vars.clear(); //unset the task variables -- they're set and updated later
    if (q_reference.N) { 
      if(q_reference.N == 2*s->ors->getJointStateDimension()) q_reference = q_reference.sub(0, q_reference.N/2 - 1);
      s->sys.setqv(q_reference, v_reference);
    }
    else 
      s->sys.getqv0(q_reference, v_reference);
    
    //update all task variables using this ors state
    FeedbackControlTaskAbstraction *task = controllerTask->get_feedbackControlTask(this);
    if (task->requiresInit) task->initTaskVariables(*s->sys.ors);
    s->sys.setTaskVariables(task->TVs);
    task->updateTaskVariableGoals(*s->sys.ors);
    
    //=== compute motion from the task variables
    //check if a collition and limit variable are active
    bool colActive=false, limActive=false;
    uint i; TaskVariable *v;
    for_list(i, v, s->sys.vars) if (v->active) {
      //?? ist sys.vars und task->vars eigentlich das gleiche??
      if (v->type==collTVT) colActive=true;
      if (v->type==qLimitsTVT) limActive=true;
    }
    if (forceColLimTVs && (!colActive || !limActive)) HALT("SAFETY BREACH! You need an active collision and limit variable!");
    
    //compute the motion step
    arr q_old=q_reference;
    arr dq, x, x_1;
    
    //dynamic control using SOC
    x_1=q_reference; x_1.append(v_reference);
    soc::bayesianDynamicControl(s->sys, x, x_1, 0);
    q_reference = x.sub(0, q_reference.N-1);
    v_reference = x.sub(v_reference.N, -1);
    
    //perhaps fix fingers
    if (fixFingers) for (uint j=7; j<14; j++) { v_reference(j)=0.; q_reference(j)=q_old(j); }
    
    //SAFTY CHECK: too large steps?
    double step=euclideanDistance(q_reference, q_old);
    if (step>s->maxJointStep) {
      MT_MSG(" *** WARNING *** too large step -> step |dq|=" <<step);
      q_reference=q_old + (q_reference-q_old)*s->maxJointStep/step;
      v_reference *= .5*s->maxJointStep/step;
      step=euclideanDistance(q_reference, q_old);
      MT_MSG(" *** WARNING *** too large step -> scaling to |dq_new|=" <<step);
      //v_reference.setZero(); SD: making too large step warnig  use max allowed step
    }
    
    hardwareReference->set_q_reference(q_reference, this);
    hardwareReference->set_v_reference(v_reference, this);
    
    //push proxies to the geometric state
    //MT_MSG("TODO");
    /* Eigentlich spielt controller iM eine double role: als
       q_reference berechnen, und die kinematic/proxies/taskvariables
       mit ors berechnen -> 2 Prozesse?
     */
    /*if(proxiesVar){
      proxiesVar->writeAccess(this);
      listCopy(proxiesVar->proxies, ors.proxies);
      proxiesVar->deAccess(this);
      } else MT_MSG("Variable pointer not set");*/
  }
  
  if (mode==ControllerTask::done) {
    return;
  }
  
}
