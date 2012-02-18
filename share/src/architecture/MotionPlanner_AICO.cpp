#include "motion.h"
#include <MT/aico.h>

#if 0

struct sMotionPlanner{
  ors::Graph *ors;
  soc::SocSystem_Ors sys;
  OpenGL *gl;
  uint verbose;
  AICO planner;
};

MotionPlanner_AICO::MotionPlanner_AICO():Process("MotionPlanner_AICO"),
motionPlan(&p), geometricState(&g){
  s = new sMotionPlanner;
  s->verbose = 0;
  s->gl=NULL;
}

MotionPlanner_AICO::~MotionPlanner_AICO(){
  delete s;
}

void MotionPlanner_AICO::open(){
  CHECK(geo, "please set geometricState before launching MotionPrimitive");
  
  //clone the geometric state
  geo->readAccess(this);
  s->ors = geo->ors.newClone();
  geo->deAccess(this);
  if(s->verbose){
    s->gl->add(glStandardScene);
    s->gl->add(ors::glDrawGraph, s->ors);
    s->gl->camera.setPosition(5, -10, 10);
    s->gl->camera.focus(0, 0, 1);
    s->gl->camera.upright();
    s->gl->update();
  }
  
  s->sys.initBasics(s->ors, NULL, (s->verbose?s->gl:NULL),
                    MT::getParameter<uint>("reachPlanTrajectoryLength"),
                    MT::getParameter<double>("reachPlanTrajectoryDuration"),
                    false,
                    NULL);
                    //TODO: Wrate and Hrate are being pulled from MT.cfg WITHIN initBasics - that's not good
}

void MotionPlanner_AICO::close(){
  delete s->ors;
  s->ors = NULL;
}
    
void MotionPlanner_AICO::step(){
    
  if(!plan->get_hasGoal(this)){
    plan->waitForConditionSignal(.01);
    return;
  }
  
  if(plan->get_converged(this)){
    plan->waitForConditionSignal(.01);
    return;
  }

  MotionKeyframe *frame1 = plan->get_final_keyframe(this);
  MotionKeyframe *frame0 = frame1->get_previous_keyframe(this);

  arr q0 = frame0->get_q_estimate(this);
  arr qT = frame1->get_q_estimate(this);
  uint T = plan->get_steps(this);
  double tau = frame1->get_duration_estimate(this)/double(T);

  arr q;
  q.resize(T+1,q0.N);
  for(uint t=0;t<=T;t++){
    double a=double(t)/T;
    q[t] = (1.-a)*q0 + a*qT;
  }
  
  plan->writeAccess(this);
  plan->q_plan = q;
  plan->tau = tau;
  plan->converged = true;
  plan->deAccess(this);

  MT::wait(1.);  return;
  
  if(!motionPlan->get_hasGoal(this)){
    MT::wait(0.01);
    MT_MSG("TODO");
    //broadcast mechanism instead of sleep?
    //stop your own loop and let somebody else wake you up?
    return;
  }

  //pull the current world configuration from the world
  geometricState->readAccess(this);
  s->sys.ors->copyShapesAndJoints(geometricState->ors);
  geometricState->deAccess(this);

  s->planner.init_messages();
  double d=s->planner.step();

  motionPlan->writeAccess(this);
  motionPlan->q_plan = s->planner.q;
  motionPlan->tau  = s->sys.getTau();
  motionPlan->converged=(d<s->planner.tolerance);// NIKOLAY : enssure reasonable plans
  motionPlan->deAccess(this);
}

#endif