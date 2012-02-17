#include "motion.h"
#include <MT/aico.h>

struct sMotionPlanner_interpolation{
  ors::Graph *ors;
};

MotionPlanner_interpolation::MotionPlanner_interpolation(MotionPlan& p, GeometricState& g):Process("MotionPlanner_interpolation"),
        plan(&p), geo(&g){
  s = new sMotionPlanner_interpolation;
}

MotionPlanner_interpolation::~MotionPlanner_interpolation(){
  delete s;
}

void MotionPlanner_interpolation::open(){
  //clone the geometric state
  geo->readAccess(this);
  s->ors = geo->ors.newClone();
  geo->deAccess(this);
}

void MotionPlanner_interpolation::close(){ MT_MSG("NIY") }
    
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
}
