#include "motion.h"
#include <MT/aico.h>

struct sMotionPlanner_interpolation{
  ors::Graph *ors;
  soc::SocSystem_Ors sys;
  OpenGL *gl;
  uint verbose;
};

MotionPlanner_interpolation::MotionPlanner_interpolation(MotionPlan& p, GeometricState& g):Process("MotionPlanner_interpolation"),
        plan(&p), geo(&g){
  s = new sMotionPlanner_interpolation;
  s->verbose = 2;
  s->gl=NULL;
  algo=AICO_noinit;
}

MotionPlanner_interpolation::~MotionPlanner_interpolation(){
  delete s;
}

void MotionPlanner_interpolation::open(){
  //clone the geometric state
  geo->readAccess(this);
  s->ors = geo->ors.newClone();
  geo->deAccess(this);

  if(s->verbose){
    s->gl = new OpenGL;
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
}

void MotionPlanner_interpolation::close(){
  if(s->gl)  delete s->gl;   s->gl=NULL;
  if(s->ors) delete s->ors;  s->ors=NULL;
}
    
void MotionPlanner_interpolation::step(){
  
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

  //copy all task variables!
  plan->writeAccess(this);
  listClone(s->sys.vars, plan->TVs);
  plan->deAccess(this);

  arr q;
  switch(algo){
  case interpolation:{
    q.resize(T+1,q0.N);
    for(uint t=0;t<=T;t++){
      double a=double(t)/T;
      q[t] = (1.-a)*q0 + a*qT;
    }
  } break;
  case AICO_noinit:{
    q.resize(T+1,q0.N);
    for(uint t=0;t<=T;t++){
      double a=double(t)/T;
      q[t] = (1.-a)*q0 + a*qT;
    }
    AICO aico(s->sys);
    //soc::straightTaskTrajectory(s->sys, q, 0);
    aico.init_trajectory(q);
    aico.iterate_to_convergence();
    //sys.costChecks(aico.b);
    //sys.analyzeTrajectory(aico.b,true);
    q = aico.q;
  } break;
  }
  
  plan->writeAccess(this);
  plan->q_plan = q;
  plan->tau = tau;
  plan->converged = true;
  plan->deAccess(this);
}
