#include "motionHeuristics.h"
#include "taskMap_default.h"
#include "taskMap_proxy.h"
#include <Optim/optimization.h>

#include <Gui/opengl.h>

//===========================================================================
//
// the rest is on the three base routines
//

void threeStepGraspHeuristic(arr& x, MotionProblem& MP, const arr& x0, uint shapeId, uint verbose) {
  OpenGL gl;
  bindOrsToOpenGL(*MP.ors, gl);

  uint T = MP.T;
  //double duration = sys.getTau() * T;
  
  MP.setx0(x0);
  listDelete(MP.taskCosts());
  
  uint side=0;
  
  //-- optimize ignoring hand -- testing different options for aligning with the object
  if (MP.ors->shapes(shapeId)->type==ors::boxST) {
    arr cost_side(3),x_side(3,x0.N);
    for (side=0; side<3; side++) {
      setGraspGoals_PR2(MP, T, shapeId, side, 0);
      cost_side(side) = keyframeOptimizer(x, MP, false, verbose);
      listDelete(MP.taskCosts());
      if (verbose>=2) {
        displayState(x, *MP.ors, gl, STRING("posture estimate phase 0 side " <<side));
      }
      x_side[side]() = x;
    }
    cout <<"3 side costs=" <<cost_side <<endl;
    side = cost_side.minIndex();
    x = x_side[side];
  } else {
    setGraspGoals_PR2(MP, T, shapeId, side, 0);
    keyframeOptimizer(x, MP, false, verbose);
    listDelete(MP.taskCosts());
    if (verbose>=2) {
      displayState(x, *MP.ors, gl, "posture estimate phase 0");
    }
  }
  
  //-- open hand
  //x.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  x(MP.ors->getJointByName("finger_l_l")->qIndex) = 1.;
  x(MP.ors->getJointByName("finger_l_r")->qIndex) = 1.;

  if (verbose>=2) {
    displayState(x, *MP.ors, gl, "posture estimate phase 1");
  }
  
  //-- reoptimize with close hand
  setGraspGoals_PR2(MP, T, shapeId, side, 1);
  keyframeOptimizer(x, MP, true, verbose);
  //listDelete(M.vars); //DON'T delete the grasp goals - the system should keep them for the planner
  if (verbose>=1) displayState(x, *MP.ors, gl, "posture estimate phase 2");
//  M.displayCurrentState("posture estimate phase 2", false, false);
  //if (verbose>=2) M.gl->watch();
}

void setGraspGoals_Schunk(MotionProblem& MP, uint T, uint shapeId, uint side, uint phase) {
  MP.setState(MP.x0, MP.v0);;
  
  //load parameters only once!
  double positionPrec = MT::getParameter<double>("graspPlanPositionPrec");
  double oppositionPrec = MT::getParameter<double>("graspPlanOppositionPrec");
  double alignmentPrec = MT::getParameter<double>("graspPlanAlignmentPrec");
  double fingerDistPrec = MT::getParameter<double>("graspPlanFingerDistPrec");
  double colPrec = MT::getParameter<double>("graspPlanColPrec");
  double limPrec = MT::getParameter<double>("graspPlanLimPrec");
  double zeroQPrec = MT::getParameter<double>("graspPlanZeroQPrec");
  
  //set the time horizon
  CHECK(T==MP.T, "");
  
  //deactivate all variables
  MP.activateAllTaskCosts(false);
  
  //activate collision testing with target shape
  ors::Shape *target_shape = MP.ors->shapes(shapeId);
  target_shape->cont=true;
  MP.swift->initActivations(*MP.ors);
  
  //
  arr target,initial;
  ors::Vector ivec,jvec;

  //general target
  target = ARRAY(target_shape->X.pos);
  //xtarget(2) += .02; //grasp it 2cm above center
  
  //-- graspCenter -> predefined point (xtarget)
  TaskCost *c;
  c = MP.addTaskMap("graspCenter",
                   new DefaultTaskMap(posTMT, "graspCenter"));
  MP.setInterpolatingCosts(c, MotionProblem::early_restConst,
                          target, positionPrec, NoArr, -1., .8);

  //-- up: align either with cylinder axis or one of the box sides -- works good
  ivec.set(0, 1, 0);
  jvec.set(0, 0, 1);
  switch (target_shape->type) {
    case ors::cylinderST:
      target = ARR(0.);  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case ors::boxST: {
      //jrel=target_shape->X;
      if (side==1) jvec.set(0, 1, 0);
      if (side==2) jvec.set(1, 0, 0);
      target = ARR(1.);  //y-axis of m9 is aligned with one of the 3 sides of the cube
    } break;
    default: NIY;
  }
  c = MP.addTaskMap("upAlign",
                   new DefaultTaskMap(vecAlignTMT, *MP.ors, "graspCenter", ivec, target_shape->name, jvec, NoArr));
  MP.setInterpolatingCosts(c, MotionProblem::early_restConst,
                          target, alignmentPrec, NoArr, -1., .8);
  //test current state: flip if necessary
  c->map.phi(initial, NoArr, *MP.ors);
  if (initial(0)<0.) ((DefaultTaskMap*)&c->map)->ivec.set(0., -1., 0.); //flip vector to become positive


  if (phase==0) return;
  

  //-- finger tips close to surface : using ProxyTaskVariable
  uintA shapes = stringListToShapeIndices(
                   ARRAY<const char*>("tip1Shape",
                                      "tip2Shape",
                                      "tip3Shape"), MP.ors->shapes);
  shapes.append(shapeId); shapes.append(shapeId); shapes.append(shapeId);
  shapes.reshape(2,3); shapes = ~shapes;
  c = MP.addTaskMap("graspContacts", new ProxyTaskMap(vectorPTMT, shapes, .05, true));
  double grip=.8; //specifies the desired proxy value
  target = ARR(grip,grip,grip);
  MP.setInterpolatingCosts(c, MotionProblem::early_restConst,
                          target, fingerDistPrec, ARR(0.,0.,0.), 0., 0.8);
  for (uint t=0; t<=T; t++) { //interpolation: 0 up to 4/5 of the trajectory, then interpolating in the last 1/5
    if (5*t<4*T) c->y_target[t]()=0.;
    else c->y_target[t]() = (grip*double(5*t-4*T))/T;
  }
  
  //-- collisions with other objects
  shapes = ARRAY<uint>(shapeId);
  c = MP.addTaskMap("otherCollisions", new ProxyTaskMap(allExceptListedPTMT, shapes, .04, true));
  target = ARR(0.);
  MP.setInterpolatingCosts(c, MotionProblem::final_restConst, target, colPrec, target, colPrec);
  c->map.phi(initial, NoArr, *MP.ors);
  if (initial(0)>0.) { //we are in collision/proximity -> depart slowly
    double a=initial(0);
    for (uint t=0; t<=T/5; t++)
      c->y_target[t]() = a*double(T-5*t)/T;
  }
  
  //-- opposing fingers
  c = MP.addTaskMap("oppose12",
                    new DefaultTaskMap(vecAlignTMT, *MP.ors, "tipNormal1", NoVector, "tipNormal2", NoVector));
  target = ARR(-1.);
  MP.setInterpolatingCosts(c, MotionProblem::early_restConst,
                          target, oppositionPrec, ARR(0.,0.,0.), 0., 0.8);
  //M.setInterpolatingCosts(c, MotionProblem::constFinalMid, target, oppositionPrec);


  c = MP.addTaskMap("oppose13",
                    new DefaultTaskMap(vecAlignTMT, *MP.ors, "tipNormal1", NoVector, "tipNormal3", NoVector));
  target = ARR(-1.);
  MP.setInterpolatingCosts(c, MotionProblem::final_restConst, target, oppositionPrec);

  
  //MT_MSG("TODO: fingers should be in relaxed position, or aligned with surface (otherwise they remain ``hooked'' as in previous posture)");
  
  //-- limits
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  c = MP.addTaskMap("limits",
                    new DefaultTaskMap(qLimitsTMT, -1, NoVector, -1, NoVector, limits));
  target=0.;
  MP.setInterpolatingCosts(c, MotionProblem::final_restConst, target, limPrec, target, limPrec);

  //-- homing
  c = MP.addTaskMap("qitself",
                    new DefaultTaskMap(qItselfTMT));
  MP.setInterpolatingCosts(c, MotionProblem::final_restConst, target, zeroQPrec, target, zeroQPrec);
}

void setGraspGoals_PR2(MotionProblem& MP, uint T, uint shapeId, uint side, uint phase) {
  MP.setState(MP.x0, MP.v0);;

  //load parameters only once!
  double positionPrec = MT::getParameter<double>("graspPlanPositionPrec");
  double alignmentPrec = MT::getParameter<double>("graspPlanAlignmentPrec");
  double fingerDistPrec = MT::getParameter<double>("graspPlanFingerDistPrec");
  double colPrec = MT::getParameter<double>("graspPlanColPrec");
  double limPrec = MT::getParameter<double>("graspPlanLimPrec");
  double zeroQPrec = MT::getParameter<double>("graspPlanZeroQPrec");

  //set the time horizon
  CHECK(T==MP.T, "");

  //deactivate all variables
  MP.activateAllTaskCosts(false);

  //activate collision testing with target shape
  ors::Shape *target_shape = MP.ors->shapes(shapeId);
  target_shape->cont=true;
  MP.swift->initActivations(*MP.ors);

  //
  arr target,initial;
  ors::Vector ivec,jvec;

  //general target
  target = ARRAY(target_shape->X.pos);

  //-- graspCenter -> predefined point (xtarget)
  TaskCost *c;
  c = MP.addTaskMap("graspCenter",
                    new DefaultTaskMap(posTMT, *MP.ors, "graspCenter"));
  MP.setInterpolatingCosts(c, MotionProblem::early_restConst,
                          target, positionPrec, NoArr, -1., .8);

  //-- align either with cylinder axis or one of the box sides -- works good
  ivec.set(0,1,0); //we want to align the y-axis of the hand with something
  jvec.set(0,0,1);
  switch (target_shape->type) {
    case ors::cylinderST:
      target = ARR(0.);  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case ors::boxST: {
      //jrel=target_shape->X;
      if (side==1) jvec.set(0,1,0);
      if (side==2) jvec.set(1,0,0);
      target = ARR(1.);  //y-axis of m9 is aligned with one of the 3 sides of the cube
    } break;
    default: NIY;
  }
  c = MP.addTaskMap("upAlign",
                    new DefaultTaskMap(vecAlignTMT, *MP.ors, "graspCenter", ivec, target_shape->name, jvec, NoArr));
  MP.setInterpolatingCosts(c, MotionProblem::early_restConst,
                          target, alignmentPrec, NoArr, -1., .8);
  //test current state: flip if necessary
  c->map.phi(initial, NoArr, *MP.ors);
  if (initial(0)<0.) ((DefaultTaskMap*)&c->map)->ivec.set(0,-1,0); //flip vector to become positive

  if (phase==0) return;

  //-- finger tips close to surface : using ProxyTaskVariable
  uintA shapes = stringListToShapeIndices(
                   ARRAY<const char*>("tip1",
                                      "tip2"), MP.ors->shapes);
  shapes.append(shapeId); shapes.append(shapeId);
  shapes.reshape(2,2); shapes = ~shapes;
  c = MP.addTaskMap("graspContacts", new ProxyTaskMap(vectorPTMT, shapes, .1, false));
  for_(uint, i, shapes) cout <<' ' <<MP.ors->shapes(*i)->name;
  double grip=.98; //specifies the desired proxy value
  target = ARR(grip,grip);
  MP.setInterpolatingCosts(c, MotionProblem::early_restConst,
                          target, fingerDistPrec, ARR(0.,0.), 0., 0.8);
  for (uint t=.8*T; t<=T; t++) { //interpolation: 0 up to 4/5 of the trajectory, then interpolating in the last 1/5
    double a=double(t-.8*T)/(.2*T);
    c->y_target[t]() = .7*(1.-a) + grip*a;
  }

#if 1
  //-- collisions with other objects
  shapes = ARRAY<uint>(shapeId);
  c = MP.addTaskMap("otherCollisions",
                    new ProxyTaskMap(allExceptListedPTMT, shapes, .04, true));
  target = ARR(0.);
  MP.setInterpolatingCosts(c, MotionProblem::constant, target, colPrec);
//  arr initial;
  c->map.phi(initial, NoArr, *MP.ors);
  if(initial(0)>0.) { //we are in collision/proximity -> depart slowly
    for (uint t=0; t<=T/5; t++){
      double a = double(T-5*t)/T;
      c->y_target[t]() = a*initial(0);
    }
  }
#endif

//  //-- opposing fingers -- PR2-fingers always oppose!
//  c = M.addDefaultTaskMap_Shapes("oppose12", zalignTMT, "tipNormal1", NoVector, "tipNormal2", NoVector);
//  target = ARR(-1.);
//  M.setInterpolatingCosts(c, MotionProblem::constEarlyMid,
//                          target, oppositionPrec, ARR(0.,0.,0.), 0., 0.8);


  //-- homing
  c = MP.addTaskMap("qitself",
                    new DefaultTaskMap(qItselfTMT));
  MP.setInterpolatingCosts(c, MotionProblem::final_restConst, target, zeroQPrec, target, zeroQPrec);

  return;

  //MT_MSG("TODO: fingers should be in relaxed position, or aligned with surface (otherwise they remain ``hooked'' as in previous posture)");

  //-- limits
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  c = MP.addTaskMap("limits",
                    new DefaultTaskMap(qLimitsTMT, -1, NoVector, -1, NoVector, limits));
  target=0.;
  MP.setInterpolatingCosts(c, MotionProblem::final_restConst, target, limPrec, target, limPrec);

}

void reattachShape(ors::Graph& ors, SwiftInterface *swift, const char* objShape, const char* toBody);

#if 0
void setPlaceGoals(MotionProblem& MP, uint T, uint shapeId, int belowToShapeId, const arr& locationTo){
  CHECK(belowToShapeId == -1 || &locationTo == NULL, "Only one thing at a time");
  MP.setState(MP.x0, MP.v0);;
  
  double midPrec          = MT::getParameter<double>("placeMidPrec");
  double alignmentPrec    = MT::getParameter<double>("placeAlignmentPrec");
  double limPrec          = MT::getParameter<double>("placePlanLimPrec");
  double colPrec          = MT::getParameter<double>("placePlanColPrec");
  double zeroQPrec        = MT::getParameter<double>("placePlanZeroQPrec");
  double positionPrec     = MT::getParameter<double>("placePositionPrec");
  double upDownVelocity   = MT::getParameter<double>("placeUpDownVelocity");
  double upDownVelocityPrec = MT::getParameter<double>("placeUpDownVelocityPrec");

  
  //set the time horizon
  CHECK(T==MP.T, "");
  
  //deactivate all variables
  MP.activateAllTaskCosts(false);
  
  //activate collision testing with target shape
  ors::Shape *obj  = MP.ors->shapes(shapeId);
  ors::Shape *onto = NULL;
  if(belowToShapeId != -1)
     onto = MP.ors->shapes(belowToShapeId);
  if (obj->body!=MP.ors->getBodyByName("m9")){
    reattachShape(*MP.ors, NULL, obj->name, "m9");
  }
  CHECK(obj->body==MP.ors->getBodyByName("m9"), "called planPlaceTrajectory without right object in hand");
  obj->cont=true;
  if(onto) onto->cont=false;
  MP.swift->initActivations(*MP.ors, 3); //the '4' means to deactivate collisions between object and fingers (which have joint parents on level 4)
  
  TaskVariable *V;
  
  //general target
  arr xtarget;
  if(onto) {
    xtarget = ARRAY(onto->X.pos);
    xtarget(2) += .5*(onto->size[2]+obj->size[2])+.005; //above 'place' shape
  }
  else {
    xtarget = locationTo;  
  }
  
  //endeff
  V = new DefaultTaskVariable("graspCenter", *MP.ors, posTVT, "graspCenter", NULL, NoArr);
  ((DefaultTaskVariable*)V)->irel = obj->rel;
  V->updateState(*MP.ors);
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
  MP.vars().append(V);
  
  //up1
  V = new DefaultTaskVariable("up1", *MP.ors, zalignTVT, "m9", "<d(90 1 0 0)>", 0, 0, 0);
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V) -> irel.addRelativeRotationDeg(90, 1, 0, 0);
  V->updateState(*MP.ors);
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, alignmentPrec, 0., 0.);
  MP.vars().append(V);
  
  //up2
  V = new DefaultTaskVariable("up2", *MP.ors, zalignTVT, "m9", "<d( 0 1 0 0)>", 0, 0, 0);
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V)-> irel.addRelativeRotationDeg(90, 0, 1, 0);
  V->updateState(*MP.ors);
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, alignmentPrec, 0., 0.);
  MP.vars().append(V);
  
  //collisions except obj-from and obj-to
  uintA shapes = ARRAY<uint>(shapeId, shapeId, belowToShapeId);
  V = new ProxyTaskVariable("otherCollisions", *MP.ors, allExceptListedPTMT, shapes, .04, true);
  V->y_target = ARR(0.);  V->v_target = ARR(.0);
  V->y_prec = colPrec;
  V->setConstTargetsConstPrecisions(T);
  if (V->y(0)>0.) { //we are in collision/proximity -> depart slowly
    double a=V->y(0);
    for (uint t=0; t<=T/5; t++)
      V->y_target[t]() = a*double(T-5*t)/T;
  }
  MP.vars().append(V);
  
  //col lim and relax
  //TODO: there are no collisions!
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  //TODO: limits as parameter!
  V = new DefaultTaskVariable("limits", *MP.ors, qLimitsTVT, 0, 0, 0, 0, limits);
  V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  MP.vars().append(V);
  V = new DefaultTaskVariable("qitself", *MP.ors, qItselfTVT, 0, 0, 0, 0, 0);
  V->y_prec=zeroQPrec;
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setConstTargetsConstPrecisions(T);
  MP.vars().append(V);
}

void setHomingGoals(MotionProblem& M, uint T){
  M.setState(M.x0, M.v0);;
  
  //deactivate all variables
  M.activateAllTaskCosts(false);
  
  M.swift->initActivations(*M.ors);
  
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
  V = new DefaultTaskVariable("limits", *M.ors,qLimitsTVT, 0, 0, 0, 0, limits);
  V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  M.vars().append(V);

  //-- standard collisions
  double margin = .05;
  V = new DefaultTaskVariable("collision", *M.ors, collTVT, 0, 0, 0, 0, ARR(margin));
  V->y=0.;  V->y_target=0.;  V->y_prec=colPrec;  V->setConstTargetsConstPrecisions(T);
  M.vars().append(V);

  //-- qitself
  V = new DefaultTaskVariable("qitself", *M.ors, qItselfTVT, 0, 0, 0, 0, 0);
  V->updateState(*M.ors);
  V->y_target.resizeAs(V->y);  V->y_target.setZero();
  V->v=0.;  V->v_target=V->v; 
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  M.vars().append(V);
  
}
#endif


double keyframeOptimizer(arr& x, MotionProblem& MP, bool x_is_initialized, uint verbose) {

  MotionProblem_EndPoseFunction MF(MP);

  if (!x_is_initialized) x=MP.x0;

  double cost;

  optNewton(x, Convert(MF), OPT(fmin_return=&cost, verbose=verbose, stopIters=200, useAdaptiveDamping=false, damping=1e-0, maxStep=.5, stopTolerance=1e-2));

  return cost;
}


void interpolate_trajectory(arr &q, const arr& q0, const arr& qT, uint T){
  q.resize(T+1,q0.N);
  for (uint t=0; t<=T; t++) {
    double a=double(t)/T;
    q[t] = (1.-a)*q0 + a*qT;
  }
}
