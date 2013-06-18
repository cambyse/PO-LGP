#include "MotionPlanner.h"
#include "FeedbackControlTasks.h"

#include <MT/aico.h>
#include <MT/socNew.h>
#include <MT/opengl.h>
#include <unistd.h>

Process* newMotionPlanner(MotionPrimitive& m){
  return new MotionPlanner(m);
}

struct sMotionPlanner {
  enum MotionPlannerAlgo { interpolation=0, AICO_noinit } planningAlgo;
  WorkingCopy<GeometricState> geo;
  OrsSystem sys;
  OpenGL *gl;
  uint verbose;
  AICO *aico;
};

MotionPlanner::MotionPlanner(MotionPrimitive& m):Process("MotionPlanner"),
    motionPrimitive(&m){
  s = new sMotionPlanner;
  s->geo.init("GeometricState", this);
  s->gl=NULL;
  s->planningAlgo=sMotionPlanner::AICO_noinit;
  s->aico=NULL;
  listenTo(&m);
}

MotionPlanner::~MotionPlanner() {
  delete s;
}

void MotionPlanner::open() {
  s->verbose = biros().getParameter<uint>("MotionPlanner_verbose", this);
  arr W = biros().getParameter<arr>("MotionPlanner_W", this);
  uint T = biros().getParameter<uint>("MotionPlanner_TrajectoryLength", this);
  double duration = biros().getParameter<double>("MotionPlanner_TrajectoryDuration", this);
  
  //clone the geometric state
  s->geo.pull();
  
  if (s->verbose) {
    s->gl = new OpenGL("MotionPlanner");
    s->gl->add(glStandardScene);
    s->gl->add(ors::glDrawGraph, &s->geo().ors);
    s->gl->camera.setPosition(5, -10, 10);
    s->gl->camera.focus(0, 0, 1);
    s->gl->camera.upright();
    s->gl->update();
  }
  
  s->sys.initBasics(&s->geo().ors, NULL, (s->verbose?s->gl:NULL),
                    T, duration, true, &W);
  //TODO: Wrate and Hrate are being pulled from MT.cfg WITHIN initBasics - that's not good
}

void MotionPlanner::close() {
}

void MotionPlanner::step() {
  s->geo.pull();
  
  CHECK(motionPrimitive,"");
  MotionPrimitive *m = motionPrimitive;
  
  MotionPrimitive::ActionPredicate actionSymbol = m->get_action(this);
  
  if (actionSymbol==MotionPrimitive::toBeAssigned) {
    m->writeAccess(this);
    m->frame1 = m->frame0;
    m->q_plan.clear();
    m->tau = 0.;
    m->duration=0.;
    m->planConverged=false;
    m->deAccess(this);
  }
  
  if (actionSymbol==MotionPrimitive::grasp || actionSymbol==MotionPrimitive::place_location
      || actionSymbol==MotionPrimitive::place || actionSymbol == MotionPrimitive::homing
      || actionSymbol == MotionPrimitive::reach){

    if(m->get_planConverged(this)) return; // nothing to do anymore
    
    //pull start condition
    arr x0;
    m->get_frame0(x0, this);
    if(m->count==0 && !x0.N){ //assume this is the FIRST frame of all
      VAR(HardwareReference);
      x0 = _HardwareReference->get_q_reference(NULL);
      x0.append(_HardwareReference->get_v_reference(NULL));
      m->set_frame0(x0, this);
    }
    CHECK(x0.N==s->sys.get_xDim(),"You need to initialize frame0 to start pose!");
    s->sys.setx0(x0);
    //cout <<"0-state! in motion primitive\n" <<x0 <<"\n ...frame=" <<x0->frameCount <<' ' <<frame1->frameCount <<' ' <<m->frameCount <<endl;

    //-- estimate the keyframe
    arr xT;
//    if (!xT->get_converged(this)){
    if (actionSymbol==MotionPrimitive::grasp || actionSymbol == MotionPrimitive::reach) {
      uint shapeId = s->sys.getOrs().getShapeByName(m->get_objectRef1(this))->index;
      threeStepGraspHeuristic(xT, s->sys, x0, shapeId, s->verbose);
    }
    else if (actionSymbol==MotionPrimitive::place) {
      listDelete(s->sys.vars());
      uint shapeId = s->sys.getOrs().getShapeByName(m->get_objectRef1(this))->index;
      uint toId = s->sys.getOrs().getShapeByName(m->get_objectRef2(this))->index;
      setPlaceGoals(s->sys, s->sys.get_T(), shapeId, toId, NoArr);
      keyframeOptimizer(xT, s->sys, 1e-2, false, s->verbose);
    }
    else if (actionSymbol==MotionPrimitive::place_location) {
      listDelete(s->sys.vars());
      uint shapeId = s->sys.getOrs().getShapeByName(m->get_objectRef1(this))->index;
      arr location = m->get_locationRef(this);
      setPlaceGoals(s->sys, s->sys.get_T(), shapeId, -1, location);
      keyframeOptimizer(xT, s->sys, 1e-2, false, s->verbose);
    }
    else if (actionSymbol==MotionPrimitive::homing) {
      setHomingGoals(s->sys, s->sys.get_T());
      keyframeOptimizer(xT, s->sys, 1e-2, false, s->verbose);
    }

    //--push it
    m->set_frame1(xT, this);
    uint T = s->sys.get_T();
    double tau = s->sys.get_tau();
    m->set_duration(tau*T, this);
    
    //-- optimize the plan
    arr q;
    switch (s->planningAlgo) {
      case sMotionPlanner::interpolation: {
        interpolate_trajectory(q,x0,xT,T);
      } break;
      case sMotionPlanner::AICO_noinit: {
        //enforce zero velocity start/end vel
        if (!s->sys.isKinematic()) x0.subRange(x0.N/2,-1) = 0.;
        if (!s->sys.isKinematic()) xT.subRange(xT.N/2,-1) = 0.;

        //don't reuse AICO when plan in motion primitive has been cleared
        if(s->aico && m->get_q_plan(this).N==0){
          delete s->aico;
          s->aico=NULL;
        }

        if(!s->aico){
          s->aico = new AICO(s->sys);
          s->aico->fix_initial_state(x0);
          s->aico->fix_final_state(xT);
          interpolate_trajectory(q,x0,xT,T);
          s->aico->init_trajectory(q);
        } else { //we've been optimizing this before!!
          s->aico->fix_initial_state(x0);
          s->aico->fix_final_state(xT);
          s->aico->prepare_for_changed_task();
        }
        s->aico->iterate_to_convergence();
        //cout << s->aico->cost() << endl;
        m->writeAccess(this);
        m->cost = s->aico->cost();
        m->deAccess(this);

        q = s->aico->q();
        //delete s->aico;
      } break;
      default:
        HALT("no mode set!");
    }
    
    //-- output the motion primitive -- for the controller to go
    m->writeAccess(this);
    m->q_plan = q;
    m->tau = tau;
    m->planConverged = true;
    m->mode = MotionPrimitive::planned;
    if (actionSymbol==MotionPrimitive::place || actionSymbol==MotionPrimitive::place_location) m->fixFingers = true;
    if (actionSymbol==MotionPrimitive::homing || actionSymbol==MotionPrimitive::grasp || actionSymbol==MotionPrimitive::reach) m->fixFingers = false;
    m->deAccess(this);

  }
  
  if (actionSymbol==MotionPrimitive::openHand || actionSymbol==MotionPrimitive::closeHand) {
    if (m->get_planConverged(this)) { // nothing to do anymore
      return;
    }

    //-- set the motion primitive -- for the controller to go
    m->writeAccess(this);
    m->frame1 = m->frame0;
    m->duration = 3.; //TODO
    m->q_plan.clear();
    m->planConverged = true;
    m->mode = MotionPrimitive::feedback;
    if (actionSymbol==MotionPrimitive::closeHand) m->feedbackControlTask = new CloseHand_FeedbackControlTask;
    if (actionSymbol==MotionPrimitive::openHand) m->feedbackControlTask = new OpenHand_FeedbackControlTask;
    m->forceColLimTVs = false;
    m->fixFingers = false;
    m->deAccess(this);
  }
  
  //FUTURE: collaps all the task variable stuff to a single Phi
};
