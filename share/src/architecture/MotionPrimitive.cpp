#include "motion.h"
#include <DZ/aico_key_frames.h>

struct sMotionPrimitive{
  ors::Graph *ors;
  soc::SocSystem_Ors sys;
  OpenGL gl;

  void threeStepGraspHeuristic(arr& q_keyframe, uint shapeId);
};

MotionPrimitive::MotionPrimitive():Process("MotionPrimitive"){
  s = new sMotionPrimitive;
  action=NULL;
  motionPlan=NULL;
  motionKeyframe=NULL;
  geometricState=NULL;
}

MotionPrimitive::~MotionPrimitive(){
  delete s;
}

void MotionPrimitive::open(){
  CHECK(geometricState, "please set geometricState before launching MotionPrimitive");
  
  //clone the geometric state
  geometricState->readAccess(this);
  s->ors = geometricState->ors.newClone();
  geometricState->deAccess(this);
  s->gl.add(glStandardScene);
  s->gl.add(ors::glDrawGraph, s->ors);
  s->gl.camera.setPosition(5, -10, 10);
  s->gl.camera.focus(0, 0, 1);
  s->gl.camera.upright();

  s->sys.initBasics(s->ors, NULL, &s->gl,
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
  CHECK(action, "please set action before launching MotionPrimitive");
  CHECK(motionPlan, "please set plan before launching MotionPrimitive");
  CHECK(motionKeyframe, "please set keyFrame before launching MotionPrimitive");
  
  Action::ActionPredicate actionSymbol = action->get_action(this);

  if(true){//TODO: need to pull ors configuration
    geometricState->readAccess(this);
    geometricState->ors.copyShapesAndJoints(*s->sys.ors);
    geometricState->deAccess(this);
  }

  if(actionSymbol==Action::noAction){
    motionPlan->set_hasGoal(false,this);
    motionKeyframe->set_hasGoal(false,this);
  }
  
  if(actionSymbol==Action::grasp){
    uint shapeId = s->sys.ors->getShapeByName(action->get_objectRef1(this))->index;
    //estimate the key frame
    arr q_keyframe;
    s->threeStepGraspHeuristic(q_keyframe, shapeId);

    //push it
    motionKeyframe->writeAccess(this);
    motionKeyframe->q_estimate = q_keyframe;
    motionKeyframe->duration_estimate = s->sys.getDuration();
    motionKeyframe->converged = true;
    motionKeyframe->deAccess(this);

    //start planner
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

    //... to set task variables

    //FUTURE: collaps all the task variable stuff to a single Phi
};


void setNewGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase);
void delNewGraspGoals(soc::SocSystem_Ors& sys);

void sMotionPrimitive::threeStepGraspHeuristic(arr& q_keyframex, uint shapeId){
  uint T = sys.nTime();
  //double duration = sys.getTau() * T;
  
  arr q,q0;
  sys.getq0(q0);


  uint side=0;
  //uint verbose=0;
  int counter=0;

  //-- optimize ignoring hand -- testing different options for aligning with the object
  if(sys.ors->shapes(shapeId)->type==ors::boxST){
    arr cost_side(3),q_side(3,q0.N);
    for(side=0;side<3;side++){
      setNewGraspGoals(sys, T, shapeId, side, 0);
      cost_side(side) = OneStepKinematic(q, NoArr, counter, sys, 1e-2,false);
      delNewGraspGoals(sys);
      sys.displayState(NULL, NULL, "posture estimate phase 0", false);
      sys.gl->watch();
      q_side[side]() = q;
    }
    q = q_side[cost_side.minIndex()];
  }else{
    setNewGraspGoals(sys, T, shapeId, side, 0);
    OneStepKinematic(q, NoArr, counter, sys, 1e-2, false);
    delNewGraspGoals(sys);
    sys.displayState(NULL, NULL, "posture estimate phase 0", false);
  }
  
  
  //-- open hand
  q.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  sys.setx(q);
  sys.displayState(NULL, NULL, "posture estimate phase 1", false);
  sys.gl->watch();

  //-- reoptimize with close hand
  setNewGraspGoals(sys,T,shapeId, side, 1);
  OneStepKinematic(q, NoArr, counter, sys, 1e-2, true);
  delNewGraspGoals(sys);
  sys.displayState(NULL, NULL, "posture estimate phase 2", false);
  sys.gl->watch();
}


//===========================================================================
//
// grasp goals
//

//cleaned version
void setNewGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase){
  sys.setTox0();
  
  //load parameters only once!
  static bool firstTime=true;
  static double midPrec, endPrec, palmPrec, colPrec, limPrec, endVelPrec;
  if(firstTime){
    firstTime=false;
    MT::getParameter(midPrec, "reachPlanMidPrec");
    MT::getParameter(endPrec, "reachPlanEndPrec");
    MT::getParameter(palmPrec, "reachPlanPalmPrec");
    MT::getParameter(colPrec, "reachPlanColPrec");
    MT::getParameter(limPrec, "reachPlanLimPrec");
    MT::getParameter(endVelPrec, "reachPlanEndVelPrec");
  }
  
  //set the time horizon
  CHECK(T==sys.nTime(), "");
  
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
  V = new DefaultTaskVariable("graspCenter", *sys.ors, posTVT, "graspCenter", NULL, NULL);
  V->y_target = xtarget;
  V->y_prec = 1e3;
  V->setInterpolatedTargetsEndPrecisions(4*T/5, midPrec, 0.);
  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);
  
  //up: align either with cylinder axis or one of the box sides -- works good
  V=new DefaultTaskVariable("upAlign", *sys.ors, zalignTVT, "graspCenter", obj->name, arr());
  ((DefaultTaskVariable*)V)->irel.setText("<d(90 1 0 0)>");
  switch(obj->type){
    case ors::cylinderST:
      V->y_target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case ors::boxST:{
      ((DefaultTaskVariable*)V)->jrel=obj->X;
      if(side==1) ((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,1,0,0);
      if(side==2) ((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,0,1,0);
      V->y_target = 1.;  //y-axis of m9 is aligned with one of the 3 sides of the cube
    }break;
    default: NIY;
  }
  V->updateState();
  if(V->y(0)<0.) ((DefaultTaskVariable*)V)->irel.addRelativeRotationDeg(180,1,0,0); //flip vector to become positive
  V->updateState();
  V->y_prec = 1e3; //endPrec;
  //V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  V->setInterpolatedTargetsEndPrecisions(4*T/5, midPrec, 0.);
  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);

  if(phase==0) return;
  
  //finger tips close to surface : using ProxyTaskVariable
  uintA shapes = stringListToShapeIndices(
    ARRAY<const char*>("tip1Shape",
                       "tip2Shape",
                       "tip3Shape"), sys.ors->shapes);
  shapes.append(shapeId);shapes.append(shapeId);shapes.append(shapeId);
  shapes.reshape(2,3); shapes = ~shapes;
  V = new ProxyTaskVariable("graspContacts", *sys.ors, vectorCTVT, shapes, .05, true);
  double grip=.9; //specifies the desired proxy value
  V->y_target = ARR(grip,grip,grip);  V->v_target = ARR(.0,.0,.0);
  V->y_prec = colPrec;
  V->setInterpolatedTargetsEndPrecisions(T,colPrec,1e1,0.,0.);
  for(uint t=0;t<=T;t++){ //interpolation: 0 up to 4/5 of the trajectory, then interpolating in the last 1/5
    if(5*t<4*T) V->y_trajectory[t]()=0.;
    else V->y_trajectory[t]() = (grip*double(5*t-4*T))/T;
  }
  sys.vars.append(V);

  //collisions with other objects
  shapes = ARRAY(shapeId);
  V = new ProxyTaskVariable("otherCollisions", *sys.ors, allExceptListedCTVT, shapes, .04, true);
  V->y_target = ARR(0.);  V->v_target = ARR(.0);
  V->y_prec = colPrec;
  V->setConstTargetsConstPrecisions(T);
  if(V->y(0)>0.){ //we are in collision/proximity -> depart slowly
    double a=V->y(0);
    for(uint t=0;t<=T/5;t++)
      V->y_trajectory[t]() = a*double(T-5*t)/T;
  }
  sys.vars.append(V);

  //opposing fingers
  V=listFindByName(sys.vars, "oppose12");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, midPrec, endPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
  V=listFindByName(sys.vars, "oppose13");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, midPrec, endPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);

  MT_MSG("TODO: fingers should be in relaxed position, or aligned with surface (otherwise they remain ``hooked'' as in previous posture)");
  
  //col lim and relax
  V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  V=listFindByName(sys.vars, "qitself");
  V->y_prec=MT::getParameter<double>("reachPlanHomeComfort");
  V->v_prec=MT::getParameter<double>("reachPlanEndVelPrec");
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, V->y_prec, V->y_prec, midPrec, V->v_prec);
}


void delNewGraspGoals(soc::SocSystem_Ors& sys){
  sys.vars.memMove=true;
  TaskVariable *V;
  V=listFindByName(sys.vars, "graspCenter");      sys.vars.removeValue(V);  delete(V);
  V=listFindByName(sys.vars, "upAlign");          sys.vars.removeValue(V);  delete(V);
  V=listFindByName(sys.vars, "graspContacts");    if(V){ sys.vars.removeValue(V);  delete(V); }
  V=listFindByName(sys.vars, "otherCollisions");  if(V){ sys.vars.removeValue(V);  delete(V); }
}
