#include "motion.h"
#include "FeedbackControlTasks.h"

#include <MT/soc.h>
#include <MT/soc_inverseKinematics.h>
#include <hardware/hardware.h>

struct sController {
  MotionFuture *motionFuture;
  MotionPrimitive *motionPrimitive;
  HardwareReference *hardwareReference;
  //GeometricState *geo;
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

Controller::Controller():Process("MotionController") {
  s = new sController();
  birosInfo.getVariable(s->motionFuture, "MotionFuture", this);
  birosInfo.getVariable(s->motionPrimitive, "MotionPrimitive", this);
  birosInfo.getVariable(s->hardwareReference, "HardwareReference", this);
  bool listens = birosInfo.getParameter<bool>("Controller_listens", this);
  if(listens) threadListenTo(s->hardwareReference);
  s->geo.init("GeometricState", this);
  s->hardwareReference->writeAccess(this);
  s->geo().ors.getJointState(s->hardwareReference->q_reference,
                             s->hardwareReference->v_reference);
  s->hardwareReference->deAccess(this);
  //s->motionPrimitive = NULL;
  
}

Controller::~Controller() {
  delete s;
}

void Controller::open() {
  arr W = birosInfo.getParameter<arr>("Controller_W", this);
  s->tau = birosInfo.getParameter<double>("Controller_tau", this);
  s->maxJointStep = birosInfo.getParameter<double>("Controller_maxJointStep", this);
  s->followTrajectoryTimeScale = birosInfo.getParameter<double>("Controller_followTrajectoryTimeScale", this);
    
  
  //clone the geometric state
  s->geo.pull();
  
  s->sys.initBasics(&s->geo().ors, NULL, NULL,
                    1, s->tau, true, &W);
}

void Controller::close() { }

void Controller::step() {
  arr zeros(14);  zeros.setZero();
  
  s->geo.pull();

  CHECK(s->motionFuture,"");
  s->motionFuture->readAccess(this);
  if(s->motionFuture->motions.N)
    s->motionPrimitive = s->motionFuture->motions(s->motionFuture->currentFrame);
  s->motionFuture->deAccess(this);
    
  if (!s->motionPrimitive){ //no motion primitive is set! don't do anything
    s->hardwareReference->set_v_reference(zeros, this);
    return;
  }

  MotionPrimitive::MotionMode mode=s->motionPrimitive->get_mode(this);
  
  if (mode==MotionPrimitive::stop || mode==MotionPrimitive::done) {
    s->hardwareReference->set_v_reference(zeros, this);
    return;
  }
  
  if (mode==MotionPrimitive::followPlan) {
    CHECK(s->motionPrimitive, "please set motionPrimitive before launching ActionToMotionPrimitive");
    
    //-- check if converged
    if (s->motionPrimitive->get_planConverged(this)==false) {
      s->hardwareReference->set_v_reference(zeros, this);
      return;
    }
    
    //-- first compute the interpolated
    double realTime = s->motionPrimitive->get_relativeRealTimeOfController(this);
    arr q_plan = s->motionPrimitive->get_q_plan(this);
    double plan_tau = s->motionPrimitive->get_tau(this);
    
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
    
    //cout <<"Following trajectory: realTime=" <<realTime <<" step=" <<timeStep <<'+' <<inter <<endl;
    s->motionPrimitive->set_relativeRealTimeOfController(realTime, this);
    
    if (timeStep>=q_plan.d0-1) {
      s->motionPrimitive->set_mode(MotionPrimitive::done, this);
      s->motionPrimitive->set_relativeRealTimeOfController(0., this);
    }
    
    //-- now test for collision
    //MT_MSG("TODO");
    
    //-- pass to MotionReference
    s->hardwareReference->set_q_reference(q_reference, this);
  }
  
  if (mode==MotionPrimitive::feedback) {
    bool forceColLimTVs = s->motionPrimitive->get_forceColLimTVs(this);
    bool fixFingers = s->motionPrimitive->get_fixFingers(this);
    
    //pull for possible changes in the geometric state
    //MT_MSG("TODO");
    
    //update the controllers own internal ors state - pulling from MotionReference
    arr q_old = s->hardwareReference->get_q_reference(this);
    arr v_old = s->hardwareReference->get_v_reference(this);
    s->sys.vars.clear(); //unset the task variables -- they're set and updated later
    if (q_old.N >= 14) { 
      if(q_old.N == 2*s->geo().ors.getJointStateDimension()) q_old = q_old.sub(0, q_old.N/2 - 1);
      s->sys.setqv(q_old, v_old);
    } else 
      s->sys.getqv0(q_old, v_old);
    
    //update all task variables using this ors state
    FeedbackControlTaskAbstraction *task = s->motionPrimitive->get_feedbackControlTask(this);
    CHECK(task,"");
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
    s->hardwareReference->set_q_reference(q_reference, this);
    s->hardwareReference->set_v_reference(v_reference, this);
    s->geo().ors.setJointState(q_reference, v_reference);
    s->geo().ors.calcBodyFramesFromJoints();
    s->geo.push();
    
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
  
}
