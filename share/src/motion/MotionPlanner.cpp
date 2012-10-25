#include "MotionPlanner.h"
#include "FeedbackControlTasks.h"
#include "motion_internal.h"

#include <MT/aico.h>
#include <unistd.h>

Process* newMotionPlanner(MotionPrimitive& m){
  return new MotionPlanner(m);
}

struct sMotionPlanner {
  enum MotionPlannerAlgo { interpolation=0, AICO_noinit } planningAlgo;
  WorkingCopy<GeometricState> geo;
  soc::SocSystem_Ors sys;
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
  threadListenTo(&m);
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
  
  if (actionSymbol==MotionPrimitive::noAction) {
    m->writeAccess(this);
    m->frame1 = m->frame0;
    m->q_plan.clear();
    m->tau = 0.;
    m->duration=0.;
    m->planConverged=false;
    m->deAccess(this);
  }
  
  if (actionSymbol==MotionPrimitive::grasp || actionSymbol==MotionPrimitive::place_location || actionSymbol==MotionPrimitive::place || actionSymbol == MotionPrimitive::homing || actionSymbol == MotionPrimitive::reach){

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
    CHECK(x0.N==2*s->sys.qDim(),"You need to initialize frame0 to start pose!");
    s->sys.setx0(x0);
    //cout <<"0-state! in motion primitive\n" <<x0 <<"\n ...frame=" <<x0->frameCount <<' ' <<frame1->frameCount <<' ' <<m->frameCount <<endl;

    //-- estimate the keyframe
    arr xT;
//    if (!xT->get_converged(this)){
    if (actionSymbol==MotionPrimitive::grasp || actionSymbol == MotionPrimitive::reach) {
      uint shapeId = s->sys.ors->getShapeByName(m->get_objectRef1(this))->index;
      threeStepGraspHeuristic(xT, s->sys, x0, shapeId, s->verbose);
    }
    else if (actionSymbol==MotionPrimitive::place) {
      listDelete(s->sys.vars);
      uint shapeId = s->sys.ors->getShapeByName(m->get_objectRef1(this))->index;
      uint toId = s->sys.ors->getShapeByName(m->get_objectRef2(this))->index;
      setPlaceGoals(s->sys, s->sys.get_T(), shapeId, toId, NoArr);
      keyframeOptimizer(xT, s->sys, 1e-2, false, s->verbose);
    }
    else if (actionSymbol==MotionPrimitive::place_location) {
      listDelete(s->sys.vars);
      uint shapeId = s->sys.ors->getShapeByName(m->get_objectRef1(this))->index;
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
    m->set_duration(s->sys.getDuration(), this);
    
    //-- optimize the plan
    uint T = s->sys.get_T();
    double tau = s->sys.getDuration()/double(T);
    arr q;
    switch (s->planningAlgo) {
      case sMotionPlanner::interpolation: {
        interpolate_trajectory(q,x0,xT,T);
      } break;
      case sMotionPlanner::AICO_noinit: {
        //enforce zero velocity start/end vel
        if (s->sys.dynamic) x0.subRange(x0.N/2,-1) = 0.;
        if (s->sys.dynamic) xT.subRange(xT.N/2,-1) = 0.;

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


//===========================================================================
//
// the rest is on the three base routines
//

void threeStepGraspHeuristic(arr& x, soc::SocSystem_Ors& sys, const arr& x0, uint shapeId, uint verbose) {
  uint T = sys.get_T();
  //double duration = sys.getTau() * T;
  
  sys.setx0(x0);
  listDelete(sys.vars);
  
  uint side=0;
  
  //-- optimize ignoring hand -- testing different options for aligning with the object
  if (sys.ors->shapes(shapeId)->type==ors::boxST) {
    arr cost_side(3),x_side(3,x0.N);
    for (side=0; side<3; side++) {
      setGraspGoals(sys, T, shapeId, side, 0);
      cost_side(side) = keyframeOptimizer(x, sys, 1e-2, false, verbose);
      listDelete(sys.vars);
      if (verbose>=2) {
        sys.displayState(NULL, NULL, "posture estimate phase 0", false);
        //sys.gl->watch();
      }
      x_side[side]() = x;
    }
    cout <<"3 side costs=" <<cost_side <<endl;
    side = cost_side.minIndex();
    x = x_side[side];
  } else {
    setGraspGoals(sys, T, shapeId, side, 0);
    keyframeOptimizer(x, sys, 1e-2, false, verbose);
    listDelete(sys.vars);
    if (verbose>=2) {
      sys.displayState(NULL, NULL, "posture estimate phase 0", false);
     // sys.gl->watch();
    }
  }
  
  //-- open hand
  x.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  sys.setx(x);
  if (verbose>=2) {
    sys.displayState(NULL, NULL, "posture estimate phase 1", false);
    //sys.gl->watch();
  }
  
  //-- reoptimize with close hand
  setGraspGoals(sys,T,shapeId, side, 1);
  keyframeOptimizer(x, sys, 1e-2, true, verbose);
  //listDelete(sys.vars); //DON'T delete the grasp goals - the system should keep them for the planner
  if (verbose>=1) sys.displayState(NULL, NULL, "posture estimate phase 2", false);
  //if (verbose>=2) sys.gl->watch();
}

void setGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase) {
  sys.setTox0();
  
  //load parameters only once!
  double positionPrec = biros().getParameter<double>("graspPlanPositionPrec");
  double oppositionPrec = biros().getParameter<double>("graspPlanOppositionPrec");
  double alignmentPrec = biros().getParameter<double>("graspPlanAlignmentPrec");
  double fingerDistPrec = biros().getParameter<double>("graspPlanFingerDistPrec");
  double colPrec = biros().getParameter<double>("graspPlanColPrec");
  double limPrec = biros().getParameter<double>("graspPlanLimPrec");
  double zeroQPrec = biros().getParameter<double>("graspPlanZeroQPrec");
  
  //set the time horizon
  CHECK(T==sys.get_T(), "");
  
  //deactivate all variables
  activateAll(sys.vars, false);
  
  //activate collision testing with target shape
  ors::Shape *obj = sys.ors->shapes(shapeId);
  obj->cont=true;
  sys.swift->initActivations(*sys.ors);
  
  TaskVariable *V;
  
  //general target
  arr xtarget(obj->X.pos.p, 3);
  //xtarget(2) += .02; //grasp it 2cm above center
  
  // graspCenter -> predefined point (xtarget)
  V = new DefaultTaskVariable("graspCenter", *sys.ors, posTVT, "graspCenter", NULL, NoArr);
  V->y_target = xtarget;
  V->y_prec = positionPrec;
  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., 0.);
  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);
  
  //up: align either with cylinder axis or one of the box sides -- works good
  V=new DefaultTaskVariable("upAlign", *sys.ors, zalignTVT, "graspCenter", obj->name, NoArr);
  ((DefaultTaskVariable*)V)->irel.setText("<d(90 1 0 0)>");
  switch (obj->type) {
    case ors::cylinderST:
      V->y_target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case ors::boxST: {
      ((DefaultTaskVariable*)V)->jrel=obj->X;
      if (side==1)((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,1,0,0);
      if (side==2)((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,0,1,0);
      V->y_target = 1.;  //y-axis of m9 is aligned with one of the 3 sides of the cube
    } break;
    default: NIY;
  }
  V->updateState(*sys.ors);
  if (V->y(0)<0.)((DefaultTaskVariable*)V)->irel.addRelativeRotationDeg(180,1,0,0); //flip vector to become positive
  V->updateState(*sys.ors);
  V->y_prec = alignmentPrec;
  //V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., 0.);
  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);
  
  if (phase==0) return;
  
  //finger tips close to surface : using ProxyTaskVariable
  uintA shapes = stringListToShapeIndices(
                   ARRAY<const char*>("tip1Shape",
                                      "tip2Shape",
                                      "tip3Shape"), sys.ors->shapes);
  shapes.append(shapeId); shapes.append(shapeId); shapes.append(shapeId);
  shapes.reshape(2,3); shapes = ~shapes;
  V = new ProxyTaskVariable("graspContacts", *sys.ors, vectorCTVT, shapes, .05, true);
  double grip=.8; //specifies the desired proxy value
  V->y_target = ARR(grip,grip,grip);  V->v_target = ARR(.0,.0,.0);
  V->y_prec = fingerDistPrec;
  V->setInterpolatedTargetsEndPrecisions(T,colPrec,fingerDistPrec,0.,0.);
  for (uint t=0; t<=T; t++) { //interpolation: 0 up to 4/5 of the trajectory, then interpolating in the last 1/5
    if (5*t<4*T) V->y_trajectory[t]()=0.;
    else V->y_trajectory[t]() = (grip*double(5*t-4*T))/T;
  }
  sys.vars.append(V);
  
  //collisions with other objects
  shapes = ARRAY<uint>(shapeId);
  V = new ProxyTaskVariable("otherCollisions", *sys.ors, allExceptListedCTVT, shapes, .04, true);
  V->y_target = ARR(0.);  V->v_target = ARR(.0);
  V->y_prec = colPrec;
  V->setConstTargetsConstPrecisions(T);
  if (V->y(0)>0.) { //we are in collision/proximity -> depart slowly
    double a=V->y(0);
    for (uint t=0; t<=T/5; t++)
      V->y_trajectory[t]() = a*double(T-5*t)/T;
  }
  sys.vars.append(V);
  
  //opposing fingers
  V = new DefaultTaskVariable("oppose12", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip2", "<d( 90 1 0 0)>", 0);
  V->y_target = ARR(-1.);  V->v_target = ARR(0.);
  V->y_prec=oppositionPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., oppositionPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);
  V = new DefaultTaskVariable("oppose13", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip3", "<d( 90 1 0 0)>", 0);
  V->y_target = ARR(-1.);  V->v_target = ARR(0.);
  V->y_prec=oppositionPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., oppositionPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);
  
  //MT_MSG("TODO: fingers should be in relaxed position, or aligned with surface (otherwise they remain ``hooked'' as in previous posture)");
  
  //col lim and relax
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  //TODO: limits as parameter!
  V = new DefaultTaskVariable("limits", *sys.ors, qLimitsTVT, 0, 0, 0, 0, limits);
  V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  sys.vars.append(V);
  V = new DefaultTaskVariable("qitself", *sys.ors, qItselfTVT, 0, 0, 0, 0, 0);
  V->y_prec=zeroQPrec;
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setConstTargetsConstPrecisions(T);
  sys.vars.append(V);
}

void reattachShape(ors::Graph& ors, SwiftInterface *swift, const char* objShape, const char* toBody);

void setPlaceGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, int belowToShapeId, const arr& locationTo){
  CHECK(belowToShapeId == -1 || &locationTo == NULL, "Only one thing at a time");
  sys.setTox0();
  
  double midPrec          = biros().getParameter<double>("placeMidPrec");
  double alignmentPrec    = biros().getParameter<double>("placeAlignmentPrec");
  double limPrec          = biros().getParameter<double>("placePlanLimPrec");
  double colPrec          = biros().getParameter<double>("placePlanColPrec");
  double zeroQPrec        = biros().getParameter<double>("placePlanZeroQPrec");
  double positionPrec     = biros().getParameter<double>("placePositionPrec");
  double upDownVelocity   = biros().getParameter<double>("placeUpDownVelocity");
  double upDownVelocityPrec = biros().getParameter<double>("placeUpDownVelocityPrec");

  
  //set the time horizon
  CHECK(T==sys.get_T(), "");
  
  //deactivate all variables
  activateAll(sys.vars, false);
  
  //activate collision testing with target shape
  ors::Shape *obj  = sys.ors->shapes(shapeId);
  ors::Shape *onto = NULL;
  if(belowToShapeId != -1)
     onto = sys.ors->shapes(belowToShapeId);
  if (obj->body!=sys.ors->getBodyByName("m9")){
    reattachShape(*sys.ors, NULL, obj->name, "m9");
  }
  CHECK(obj->body==sys.ors->getBodyByName("m9"), "called planPlaceTrajectory without right object in hand");
  obj->cont=true;
  if(onto) onto->cont=false;
  sys.swift->initActivations(*sys.ors, 3); //the '4' means to deactivate collisions between object and fingers (which have joint parents on level 4)
  
  TaskVariable *V;
  
  //general target
  arr xtarget;
  if(onto) {
    xtarget.setCarray(onto->X.pos.p, 3);
    xtarget(2) += .5*(onto->size[2]+obj->size[2])+.005; //above 'place' shape
  }
  else {
    xtarget = locationTo;  
  }
  
  //endeff
  V = new DefaultTaskVariable("graspCenter", *sys.ors, posTVT, "graspCenter", NULL, NoArr);
  ((DefaultTaskVariable*)V)->irel = obj->rel;
  V->updateState(*sys.ors);
  V->y_target = xtarget;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, positionPrec, 0., 0.);
  //special: condition effector velocities:
  uint t, M=T/8;
  for(t=0; t<M; t++){
    V -> v_trajectory[t]() = (1./M*t)*ARR(0., 0., upDownVelocity);
    V -> v_prec_trajectory(t) = upDownVelocityPrec;
  }
  for(t=T-M; t<T; t++){
    V -> v_trajectory[t]() = (1./M*(T-t))*ARR(0., 0., -upDownVelocity); //0.2
    V -> v_prec_trajectory(t) = upDownVelocityPrec; // 1e1
  }
  sys.vars.append(V);
  
  //up1
  V = new DefaultTaskVariable("up1", *sys.ors, zalignTVT, "m9", "<d(90 1 0 0)>", 0, 0, 0);
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V) -> irel.addRelativeRotationDeg(90, 1, 0, 0);
  V->updateState(*sys.ors);
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, alignmentPrec, 0., 0.);
  sys.vars.append(V);
  
  //up2
  V = new DefaultTaskVariable("up2", *sys.ors, zalignTVT, "m9", "<d( 0 1 0 0)>", 0, 0, 0);
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V)-> irel.addRelativeRotationDeg(90, 0, 1, 0);
  V->updateState(*sys.ors);
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, alignmentPrec, 0., 0.);
  sys.vars.append(V);
  
  //collisions except obj-from and obj-to
  uintA shapes = ARRAY<uint>(shapeId, shapeId, belowToShapeId);
  V = new ProxyTaskVariable("otherCollisions", *sys.ors, allExceptListedCTVT, shapes, .04, true);
  V->y_target = ARR(0.);  V->v_target = ARR(.0);
  V->y_prec = colPrec;
  V->setConstTargetsConstPrecisions(T);
  if (V->y(0)>0.) { //we are in collision/proximity -> depart slowly
    double a=V->y(0);
    for (uint t=0; t<=T/5; t++)
      V->y_trajectory[t]() = a*double(T-5*t)/T;
  }
  sys.vars.append(V);
  
  //col lim and relax
  //TODO: there are no collisions!
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  //TODO: limits as parameter!
  V = new DefaultTaskVariable("limits", *sys.ors, qLimitsTVT, 0, 0, 0, 0, limits);
  V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  sys.vars.append(V);
  V = new DefaultTaskVariable("qitself", *sys.ors, qItselfTVT, 0, 0, 0, 0, 0);
  V->y_prec=zeroQPrec;
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setConstTargetsConstPrecisions(T);
  sys.vars.append(V);
}

void setHomingGoals(soc::SocSystem_Ors& sys, uint T){
  sys.setTox0();
  
  //deactivate all variables
  activateAll(sys.vars, false);
  
  sys.swift->initActivations(*sys.ors);
  
  TaskVariable *V;
  
  //general target
  double midPrec, endPrec, limPrec, colPrec;
  MT::getParameter(midPrec, "homingPlanMidPrec");
  MT::getParameter(endPrec, "homingPlanEndPrec");
  MT::getParameter(limPrec, "homingPlanLimPrec");
  MT::getParameter(colPrec, "homingPlanColPrec");

  
  //-- limits
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  V = new DefaultTaskVariable("limits", *sys.ors, qLimitsTVT, 0, 0, 0, 0, limits);
  V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  sys.vars.append(V);

  //-- standard collisions
  double margin = .05;
  V = new DefaultTaskVariable("collision", *sys.ors, collTVT, 0, 0, 0, 0, ARR(margin));
  V->y=0.;  V->y_target=0.;  V->y_prec=colPrec;  V->setConstTargetsConstPrecisions(T);
  sys.vars.append(V);

  //-- qitself
  V = new DefaultTaskVariable("qitself", *sys.ors, qItselfTVT, 0, 0, 0, 0, 0);
  V->updateState(*sys.ors);
  V->y_target.resizeAs(V->y);  V->y_target.setZero();
  V->v=0.;  V->v_target=V->v; 
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  sys.vars.append(V);
  
}

//From Dmitry
void decomposeMatrix(arr& A1,arr& A2,arr A) { // returns diagonal blocks of equal size
  int dim = sqrt(A.N)/2;
  A.resize(2*dim,2*dim);
  A1.resize(dim,dim);
  A2.resize(dim,dim);
  for (int i=0; i<dim; i++)
    for (int j=0; j<dim; j++) {
      A1(i,j) = A(i,j);
      A2(i,j) = A(dim+i,dim+j);
    }
}

double SumOfRow(int p,int k) { // sum of geometric series
  double sum=0;
  switch (k) {
    case 0: sum=p;  break;
    case 1: sum=p*(p+1)/2.0;  break;
    case 2: sum=p*(p+1)*(2.0*p+1)/6.0;  break;
    default: NIY;
  }
  if (sum>0)
    return sum;
  else return 1;
}


double keyframeOptimizer(arr& x, soc::SocSystemAbstraction& sys, double stopTolerance, bool x_is_initialized, uint verbose) {
  arr sqrtWinv,x0;
  
  if (!sys.dynamic) {
    arr W;
    sys.getControlCosts(W, NoArr, 0);
    arr wdiag;
    getDiag(wdiag, W);
    wdiag *= double(sys.get_T());
    for (uint i=0; i<wdiag.N; i++) wdiag(i) = 1./sqrt(wdiag(i));
    sqrtWinv = diag(wdiag);
  } else {
    //From Dmitry
    double T = sys.get_T();
    //control costs
    arr Hinv,HrateInv;
    sys.getControlCosts(NoArr, Hinv, 0);
    HrateInv = Hinv*sys.getTau();

    //dynamics noise
    arr A,a,B,Q,Qrate,Q1,Q2;
    sys.getDynamics(A,a,B,Q,0);
    Qrate = Q/sys.getTau();
    decomposeMatrix(Q1,Q2,Q);

    double tau = sys.getDuration();// tau is basically = time
    double tau2=tau*tau;
    int dim=sqrt(Q.N)/2;
    
    arr I,Z,AT,Zv;
    I.setId(dim); Z.resize(dim,dim); Z.setZero();
    AT.setBlockMatrix(I,sys.getDuration()*I,Z,I);  // A to the power of T
    
    double S0 = SumOfRow(T,0); double S1 = SumOfRow(T-1,1); double S2 = SumOfRow(T-1,2);  // sums of geometric series
    arr sigma1,sigma2,sigma3,sigma4; // Blocks of sigma matrix
    sigma1 = tau2*tau*HrateInv*(S0+2.0*S1 + S2)/pow(T,3) + tau2*tau*Q2*S2/pow(T,3)+ tau*S0*Q1/T;
    sigma2 = tau2*HrateInv*(S0+S1)/pow(T,2) + tau2*S1*Q2/pow(T,2);
    sigma3 = sigma2;
    sigma4 = tau*S0*(HrateInv + Q2)/T;
    
    arr sumA,sumAinv;
    sumA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4);
    inverse_SymPosDef(sumAinv,sumA);
    //suma= AT*x0;
    lapack_cholesky(sqrtWinv, sumAinv);
  }
  
  sys.get_x0(x0);
  if (!x_is_initialized) x=x0;
  
  struct MyOptimizationProblem:VectorFunction {
    soc::SocSystemAbstraction *sys;
    arr sqrtWinv,x0;
    bool verbose;
    
    void   fv(arr& Phi, arr& J, const arr& x) {
      sys->setx(x);
      if (verbose) {
        sys->displayState(NULL, NULL, "posture", true);
        sys->gl->watch();
      }
      sys->getTaskCostTerms(Phi, J, x, sys->get_T());
      Phi.append(sqrtWinv*(x-x0));
      if (&J) J.append(sqrtWinv);
    }
  } F;
  F.sys = &sys;
  F.sqrtWinv = sqrtWinv;
  F.x0=x0;
  F.verbose=false;
  if (verbose>=3) checkJacobian(F, x, 1e-6);
  F.verbose = verbose>=3;
  
  double cost;
  optOptions opt;
  opt.fmin_return=&cost;
  opt.stopTolerance=1e-2;
  opt.stopEvals=100;
  opt.initialDamping=1.;
  opt.maxStep=.5;
  opt.verbose=verbose?verbose-1:0;
  optGaussNewton(x, F, opt);
  
  return cost;
}


void interpolate_trajectory(arr &q, const arr& q0, const arr& qT, uint T){
  q.resize(T+1,q0.N);
  for (uint t=0; t<=T; t++) {
    double a=double(t)/T;
    q[t] = (1.-a)*q0 + a*qT;
  }
}
