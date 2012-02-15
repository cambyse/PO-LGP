#include "motion.h"
#include <MT/aico.h>

struct sMotionPlanner_interpolation{
  ors::Graph *ors;
  OpenGL gl;
  uint verbose;
};

MotionPlanner_interpolation::MotionPlanner_interpolation():Process("MotionPlanner_interpolation"){
  s = new sMotionPlanner_interpolation;
  motionPlan=NULL;
  geometricState=NULL;
  s->verbose = 2;
}

MotionPlanner_interpolation::~MotionPlanner_interpolation(){
  delete s;
}

void MotionPlanner_interpolation::open(){
  //clone the geometric state
  geometricState->readAccess(this);
  s->ors = geometricState->ors.newClone();
  geometricState->deAccess(this);
  s->gl.add(glStandardScene);
  s->gl.add(ors::glDrawGraph, s->ors);
  s->gl.camera.setPosition(5, -10, 10);
  s->gl.camera.focus(0, 0, 1);
  s->gl.camera.upright();
  s->gl.update();
}

void MotionPlanner_interpolation::close(){ MT_MSG("NIY") }
    
void MotionPlanner_interpolation::step(){
  MotionKeyframe *final_keyframe = motionPlan->get_final_keyframe(this);
  MotionKeyframe *previous_keyframe = final_keyframe->get_previous_keyframe(this);

  arr q0 = previous_keyframe->get_q_estimate(this);
  arr qT = final_keyframe->get_q_estimate(this);
  uint T = motionPlan->get_steps(this);
  double tau = final_keyframe->get_duration_estimate(this)/double(T);

  arr q;
  q.resize(T+1,q0.N);
  for(uint t=0;t<=T;t++){
    double a=double(t)/T;
    q[t] = a*q0 + (1.-a)*qT;
  }
  
  motionPlan->set_q_plan(q, this);
  motionPlan->set_tau(tau, this);
  motionPlan->set_converged(true, this);
    
  if(s->verbose>=2){
    for(uint t=0;t<=T;t++){
      s->ors->setJointState(q[t]);
      s->gl.update();
    }
  }

}
