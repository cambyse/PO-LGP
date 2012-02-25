#include "motion.h"
#include <MT/aico.h>
#include <MT/biros_internal.h>

struct sMotionPlanner_interpolation {
  ors::Graph *ors;
  soc::SocSystem_Ors sys;
  OpenGL *gl;
  uint verbose;
};

MotionPlanner::MotionPlanner():Process("MotionPlanner_interpolation") {
  s = new sMotionPlanner_interpolation;
  birosInfo.getVariable(plan, "MotionPlan", this);
  birosInfo.getVariable(geo, "GeometricState", this);
  s->gl=NULL;
  algo=AICO_noinit;
}

MotionPlanner::~MotionPlanner() {
  delete s;
}

void MotionPlanner::open() {
  s->verbose = birosInfo.getParameter<uint>("MotionPlanner_verbose", this);
  arr W = birosInfo.getParameter<arr>("MotionPlanner_W", this);
  uint T = birosInfo.getParameter<uint>("MotionPlanner_TrajectoryLength", this);
  double duration = birosInfo.getParameter<double>("MotionPlanner_TrajectoryDuration", this);
  
  //clone the geometric state
  geo->readAccess(this);
  s->ors = geo->ors.newClone();
  geo->deAccess(this);
  
  if (s->verbose) {
    s->gl = new OpenGL("MotionPlanner");
    s->gl->add(glStandardScene);
    s->gl->add(ors::glDrawGraph, s->ors);
    s->gl->camera.setPosition(5, -10, 10);
    s->gl->camera.focus(0, 0, 1);
    s->gl->camera.upright();
    s->gl->update();
  }
  
  s->sys.initBasics(s->ors, NULL, (s->verbose?s->gl:NULL),
                    T, duration, true, &W);
}

void MotionPlanner::close() {
  if (s->gl)  delete s->gl;   s->gl=NULL;
  if (s->ors) delete s->ors;  s->ors=NULL;
}

void MotionPlanner::step() {

  if (!plan->get_hasGoal(this)) {
    plan->waitForConditionSignal(.01);
    return;
  }
  
  if (plan->get_converged(this)) {
    plan->waitForConditionSignal(.01);
    return;
  }
  
  MotionKeyframe *frame1 = plan->get_final_keyframe(this);
  MotionKeyframe *frame0 = frame1->get_previous_keyframe(this);
  
  arr x0 = frame0->get_x_estimate(this);
  arr xT = frame1->get_x_estimate(this);
  uint T = plan->get_steps(this);
  double tau = frame1->get_duration_estimate(this)/double(T);
  
  //copy all task variables!
  plan->writeAccess(this);
  listClone(s->sys.vars, plan->TVs);
  plan->deAccess(this);
  
  arr q;
  switch (algo) {
    case interpolation: {
      q.resize(T+1,x0.N);
      for (uint t=0; t<=T; t++) {
        double a=double(t)/T;
        q[t] = (1.-a)*x0 + a*xT;
      }
    } break;
    case AICO_noinit: {
      AICO aico(s->sys);
      if (s->sys.dynamic) x0.subRange(x0.N/2,-1) = 0.;
      if (s->sys.dynamic) xT.subRange(xT.N/2,-1) = 0.;
      aico.fix_initial_state(x0);
      aico.fix_final_state(xT);
      aico.iterate_to_convergence();
      q = aico.q;
    } break;
  }
  
  plan->writeAccess(this);
  plan->q_plan = q;
  plan->tau = tau;
  plan->converged = true;
  plan->deAccess(this);
}
