#include "motion.h"
//#include <DZ/aico_key_frames.h>

struct sMotionPrimitive{
  ors::Graph *ors;
  soc::SocSystem_Ors sys;
  OpenGL *gl;
  uint verbose;

  void threeStepGraspHeuristic(arr& q_keyframe, const arr& q0, uint shapeId);
};

MotionPrimitive::  MotionPrimitive(Action& a, MotionKeyframe& f0, MotionKeyframe& f1, MotionPlan& p, GeometricState& g):Process("MotionPrimitive"),
action(&a), frame0(&f0), frame1(&f1), plan(&p), geo(&g){
  s = new sMotionPrimitive;
  s->verbose = 0;
  s->gl=NULL;
  s->ors=NULL;
}

MotionPrimitive::~MotionPrimitive(){
  delete s;
}

void MotionPrimitive::open(){
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

void MotionPrimitive::close(){
  delete s->ors;
  s->ors = NULL;
}
    
void MotionPrimitive::step(){
  Action::ActionPredicate actionSymbol = action->get_action(this);

  if(true){//TODO: need to pull ors configuration
    geo->writeAccess(this); //BAD!
    geo->ors.copyShapesAndJoints(*s->sys.ors);
    geo->deAccess(this);
  }

  if(actionSymbol==Action::noAction){
    plan->set_hasGoal(false,this);
  }
  
  if(actionSymbol==Action::grasp){
    uint shapeId = s->sys.ors->getShapeByName(action->get_objectRef1(this))->index;

    if(frame1->get_converged(this)){
      action->waitForConditionSignal(.01);
      return;
    }
  
    //pull start condition
    arr q0;
    frame0->get_q_estimate(q0, this);
    if(!q0.N){
      s->sys.getq0(q0);
      frame0->set_q_estimate(q0, this);
      frame0->set_duration_estimate(0., this);
    }
    
    //estimate the key frame
    arr q_keyframe;
    s->threeStepGraspHeuristic(q_keyframe, q0, shapeId);

    //push it
    frame1->writeAccess(this);
    frame1->q_estimate = q_keyframe;
    frame1->duration_estimate = s->sys.getDuration();
    frame1->previous_keyframe = frame0;
    frame1->converged = true;
    frame1->deAccess(this);

    //-- start planner
    plan->writeAccess(this);
    //info on the plan: steps, duration, boundary
    plan->hasGoal =true;
    plan->steps = s->sys.nTime();
    plan->tau = s->sys.getDuration() / plan->steps;
    plan->final_keyframe = frame1;
    //details on the task costs
    listClone(plan->TVs, s->sys.vars);
    plan->deAccess(this);
    //TODO: much more!
    //setGraspGoals(s->sys, s->sys.nTime(), goalVar->graspShape);
  }
    
  if(actionSymbol==Action::place){
    //setPlaceGoals(*sys, sys->nTime(), goalVar->graspShape, goalVar->belowFromShape, goalVar->belowToShape);
      //arr q;
      //soc::straightTaskTrajectory(*sys, q, 0);
      //planner.init_trajectory(q);
  }
  
  if(actionSymbol==Action::home){
    //setHomingGoals(*sys, sys->nTime(), goalVar->graspShape, goalVar->belowToShape);
    //arr q;
    //soc::straightTaskTrajectory(*sys, q, 1); //task id is q!!!
    //planner.init_trajectory(q);
  }
  
  action->waitForConditionSignal(.01);
    //... to set task variables

    //FUTURE: collaps all the task variable stuff to a single Phi
};


void setNewGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase);
double KeyframeOptimizer(arr& q, soc::SocSystemAbstraction& sys, double stopTolerance, bool q_is_initialized, uint verbose);

void sMotionPrimitive::threeStepGraspHeuristic(arr& q, const arr& q0, uint shapeId){
  uint T = sys.nTime();
  //double duration = sys.getTau() * T;
  
  sys.setq0(q0);
  listDelete(sys.vars);

  uint side=0;

  //-- optimize ignoring hand -- testing different options for aligning with the object
  if(sys.ors->shapes(shapeId)->type==ors::boxST){
    arr cost_side(3),q_side(3,q0.N);
    for(side=0;side<3;side++){
      setNewGraspGoals(sys, T, shapeId, side, 0);
      cost_side(side) = KeyframeOptimizer(q, sys, 1e-2, false, verbose);
      listDelete(sys.vars);
      if(verbose>=2){
        sys.displayState(NULL, NULL, "posture estimate phase 0", false);
        sys.gl->watch();
      }
      q_side[side]() = q;
    }
    q = q_side[cost_side.minIndex()];
  }else{
    setNewGraspGoals(sys, T, shapeId, side, 0);
    KeyframeOptimizer(q, sys, 1e-2, false, verbose);
    listDelete(sys.vars);
    if(verbose>=2){
      sys.displayState(NULL, NULL, "posture estimate phase 0", false);
      sys.gl->watch();
    }
  }
  
  
  //-- open hand
  q.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  sys.setx(q);
  if(verbose>=2){
    sys.displayState(NULL, NULL, "posture estimate phase 1", false);
    sys.gl->watch();
  }

  //-- reoptimize with close hand
  setNewGraspGoals(sys,T,shapeId, side, 1);
  KeyframeOptimizer(q, sys, 1e-2, true, verbose);
  //listDelete(sys.vars); //DON'T delete the grasp goals - the system should keep them for the planner
  if(verbose>=1) sys.displayState(NULL, NULL, "posture estimate phase 2", false);
  if(verbose>=2) sys.gl->watch();
}


double KeyframeOptimizer(arr& q, soc::SocSystemAbstraction& sys, double stopTolerance, bool q_is_initialized, uint verbose){
  arr H,Q,q0;
  sys.getH(H,0); //H_step
  sys.getQ(Q,0); //Q_step

  arr wdiag;
  getDiag(wdiag, Q+H);
  wdiag *= double(sys.nTime());
  for(uint i=0;i<wdiag.N;i++) wdiag(i) = 1./sqrt(wdiag(i));
  
  sys.getq0(q0);
  if(!q_is_initialized) q=q0;
  
  struct MyOptimizationProblem:VectorFunction{
    soc::SocSystemAbstraction *sys;
    arr wdiag,q0;
    bool verbose;
    
    void   fv(arr& Phi, arr& J, const arr& q){
      sys->setq(q);
      if(verbose){
        sys->displayState(NULL, NULL, "posture", true);
        sys->gl->watch();
      }
      sys->getTaskCostTerms(Phi, J, q, sys->nTime());
      Phi.append(wdiag%(q-q0));
      if(&J) J.append(diag(wdiag));
    }
  } F;
  F.sys = &sys;
  F.wdiag = wdiag;
  F.q0=q0;
  F.verbose=false;
  if(verbose>=3) checkJacobian(F, q, 1e-6);
  F.verbose = verbose>=3;

  double cost;
  optOptions opt;
  opt.fmin_return=&cost;
  opt.stopTolerance=1e-2;
  opt.stopEvals=100;
  opt.initialDamping=1.;
  opt.maxStep=.5;
  opt.verbose=verbose?verbose-1:0;
  optGaussNewton(q, F, opt);
  
  return cost;
}

//===========================================================================
//
// grasp goals
//

