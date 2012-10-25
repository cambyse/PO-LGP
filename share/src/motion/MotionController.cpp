#include "motion_internal.h"
#include "FeedbackControlTasks.h"

#include <MT/soc.h>
#include <MT/soc_inverseKinematics.h>
#include <hardware/hardware.h>

Process* newMotionController(HardwareReference* hw, MotionPrimitive* mp, MotionFuture* mf){
  return new MotionController(hw, mp, mf);
}

struct sMotionController {
  WorkingCopy<GeometricState> geo;
  
  //ors::Graph *ors;
  soc::SocSystem_Ors sys;
  
  double tau;
  double maxJointStep;
  double followTrajectoryTimeScale;
  
  /*
  bool useBwdMsg, forceColLimTVs, fixFingers;
  arr bwdMsg_v, bwdMsg_Vinv; //optional: backward messages from a planner
  double maxJointStep; //computeMotionFromTaskVariables will generate a null-step if this limit is exceeded
  
  //OUTPUT
  arr q_reference, v_reference; //, q_orsInit;  //the SIMULATION state (the modules buffer double states, simulation is synchronized with modules in the loop)
  arr q_home; //posture as loaded from the ors file
  */
};

MotionController::MotionController(HardwareReference* a, MotionPrimitive* b, MotionFuture* c)
:Process("MotionController"), hardwareReference(a), motionPrimitive(b), motionFuture(c) {
  s = new sMotionController();
  if(!hardwareReference) biros().getVariable(hardwareReference, "HardwareReference", this);
  if(!motionPrimitive)   biros().getVariable(motionPrimitive, "MotionPrimitive", this);
  if(!motionFuture)      biros().getVariable(motionFuture, "MotionFuture", this);
  s->geo.init("GeometricState", this);
  hardwareReference->writeAccess(this);
  s->geo().ors.getJointState(hardwareReference->q_reference,
                             hardwareReference->v_reference);
  hardwareReference->deAccess(this);
}

MotionController::~MotionController() {
  delete s;
}

void MotionController::open() {
  arr W = biros().getParameter<arr>("MotionController_W", this);
  s->tau = biros().getParameter<double>("MotionController_tau", this);
  s->maxJointStep = biros().getParameter<double>("MotionController_maxJointStep", this);
  s->followTrajectoryTimeScale = biros().getParameter<double>("MotionController_followTrajectoryTimeScale", this);
    
  
  //clone the geometric state
  s->geo.pull();
  
  s->sys.initBasics(&s->geo().ors, NULL, NULL,
                    1, s->tau, true, &W);
}

void MotionController::close() { }

void MotionController::step() {
  arr zeros(14);  zeros.setZero();
  
  s->geo.pull();

  if(motionFuture){ //this is hooked to a motionFuture...
    motionFuture->readAccess(this);
    if(motionFuture->motions.N) {
      motionPrimitive = motionFuture->motions(motionFuture->currentFrame);
    }
    motionFuture->deAccess(this);
  }

  if (!motionPrimitive){ //no motion primitive is set! don't do anything
    hardwareReference->writeAccess(this);
    hardwareReference->v_reference.setZero();
    hardwareReference->motionPrimitiveRelativeTime = 0.;
    hardwareReference->deAccess(this);
    //MT_MSG("no motion primitive set -> controller stays put");
    return;
  }

  MotionPrimitive::MotionMode mode=motionPrimitive->get_mode(this);
  
  if (mode==MotionPrimitive::none || mode==MotionPrimitive::done) { //nothing to do -> stop
    hardwareReference->writeAccess(this);
    hardwareReference->v_reference.setZero();
    hardwareReference->motionPrimitiveRelativeTime = 0.;
    hardwareReference->deAccess(this);
    return;
  }
  
  if (mode==MotionPrimitive::planned) {
    CHECK(motionPrimitive, "please set motionPrimitive before launching MotionPlanner");
    
    bool fixFingers = motionPrimitive->get_fixFingers(this);

    //-- check if converged
    if (motionPrimitive->get_planConverged(this)==false) {
      hardwareReference->writeAccess(this);
      hardwareReference->v_reference.setZero();
      hardwareReference->motionPrimitiveRelativeTime = 0.;
      hardwareReference->deAccess(this);
      return;
    }

    //-- get current robot state
    hardwareReference->readAccess(this);
    arr q_old = hardwareReference->q_reference;
    arr v_old = hardwareReference->v_reference;
    arr q_real = q_old;
    arr v_real = v_old;
    if(hardwareReference->q_real.N)  for(uint i=7;i<14;i++) q_real(i) = hardwareReference->q_real(i);
    if(hardwareReference->v_real.N)  for(uint i=7;i<14;i++) v_real(i) = hardwareReference->v_real(i);
    hardwareReference->deAccess(this);

    //-- first compute the interpolated
    double realTime = hardwareReference->get_motionPrimitiveRelativeTime(this);
    arr q_plan = motionPrimitive->get_q_plan(this);
    double plan_tau = motionPrimitive->get_tau(this);
    
    //where to interpolate
    realTime += s->followTrajectoryTimeScale * s->tau;
    uint timeStep= realTime/plan_tau;
    double inter = realTime/plan_tau - (double)timeStep;  //same as fmod
    
    //do interpolation
    arr q_reference;
    if (timeStep+1<q_plan.d0) { //ok, time step is within the plan
      q_reference = (1.-inter)*q_plan[timeStep] + inter*q_plan[timeStep+1];
    } else { //time step is beyond the horizon of the plan
      q_reference = q_plan[q_plan.d0-1];
    }
    
    
    if (timeStep>=q_plan.d0-1) {
      motionPrimitive->set_mode(MotionPrimitive::done, this);
      hardwareReference->set_motionPrimitiveRelativeTime(0., this);
    }
    
    //-- now test for collision
    //MT_MSG("TODO");
    

    //-- test for large step
    double step=euclideanDistance(q_reference, q_old);
    if (step>s->maxJointStep) {
      MT_MSG(" *** WARNING *** too large step -> step |dq|=" <<step);
      q_reference = q_old + (q_reference-q_old)*s->maxJointStep/step;
      step=euclideanDistance(q_reference, q_old);
      MT_MSG(" *** WARNING *** too large step -> scaling to |dq_new|=" <<step);
    }

    //-- compute v_reference
    double Kp = .1/s->tau, Kd = 1e-3;
    arr v_reference = Kp*(q_reference-q_real) - Kd*v_real;

    //perhaps fix fingers
    if (fixFingers) for (uint j=7; j<14; j++) {
      v_reference(j)=0.; q_reference(j)=q_old(j); 
    }
    //-- pass to MotionReference
    //cout <<"Following trajectory: realTime=" <<realTime <<" step=" <<timeStep <<'+' <<inter <<endl;
    hardwareReference->writeAccess(this);
    hardwareReference->q_reference = q_reference;
    hardwareReference->v_reference = v_reference;
    hardwareReference->motionPrimitiveRelativeTime = realTime;
    hardwareReference->deAccess(this);
  }
  
  if (mode==MotionPrimitive::feedback) {
    bool forceColLimTVs = motionPrimitive->get_forceColLimTVs(this);
    bool fixFingers = motionPrimitive->get_fixFingers(this);
    
    //pull for possible changes in the geometric state
    //MT_MSG("TODO");
    
    //update the controllers own internal ors state - pulling from MotionReference
    hardwareReference->readAccess(this);
    arr q_old = hardwareReference->q_reference;
    arr v_old = hardwareReference->v_reference;
    if(hardwareReference->q_real.N)  for(uint i=7;i<14;i++) q_old(i) = hardwareReference->q_real(i); //copy real hand state!!!
    hardwareReference->deAccess(this);
    
    s->sys.vars.clear(); //unset the task variables -- they're set and updated later
    if (q_old.N >= 14) { 
      if(q_old.N == 2*s->geo().ors.getJointStateDimension()) q_old = q_old.sub(0, q_old.N/2 - 1);
      s->sys.setqv(q_old, v_old);
    } else 
      s->sys.getqv0(q_old, v_old);
    
    //update all task variables using this ors state
    FeedbackControlTaskAbstraction *task = motionPrimitive->get_feedbackControlTask(this);
    CHECK(task,"");
    if (task->requiresInit) task->initTaskVariables(*s->sys.ors);
    if (task->done){
      motionPrimitive->set_mode(MotionPrimitive::done, this);
      hardwareReference->writeAccess(this);
      hardwareReference->v_reference.setZero();
      hardwareReference->motionPrimitiveRelativeTime = 0.;
      hardwareReference->deAccess(this);
      return;
    }
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
    arr dq, x, x_1;
    
    //dynamic control using SOC
    x_1=q_old; x_1.append(v_old);
    arr q_reference, v_reference;
    soc::bayesianDynamicControl(s->sys, x, x_1, 0);
    q_reference = x.sub(0, q_old.N-1);
    v_reference = x.sub(v_old.N, -1);
    
    //perhaps fix fingers
    if (fixFingers) for (uint j=7; j<14; j++) {
      v_reference(j)=0.; q_reference(j)=q_old(j); 
    }
    
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
    s->geo.var->writeAccess(this);
    s->geo.var->ors = s->geo().ors;
    s->geo.var->deAccess(this);
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

}
