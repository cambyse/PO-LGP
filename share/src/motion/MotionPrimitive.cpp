#include "MotionPrimitive.h"
//#include <JK/utils/util.h>
//#include <DZ/aico_key_frames.h>
#include <unistd.h>

struct sMotionPrimitive {
  WorkingCopy<GeometricState> geo;
  soc::SocSystem_Ors sys;
  OpenGL *gl;
  uint verbose;
  
};

MotionPrimitive::MotionPrimitive(Action& a, MotionKeyframe& f0, MotionKeyframe& f1, MotionPlan& p):Process("MotionPrimitive"),
    action(&a), plan(&p){
  threadListenTo(action);
  s = new sMotionPrimitive;
  s->geo.init("GeometricState", this);
  s->gl=NULL;
  plan->writeAccess(this);
  plan->frame0 = &f0;
  plan->frame1 = &f1;
  plan->deAccess(this);
}

MotionPrimitive::~MotionPrimitive() {
  delete s;
}

void MotionPrimitive::open() {
  s->verbose = birosInfo.getParameter<uint>("MotionPrimitive_verbose", this);
  arr W = birosInfo.getParameter<arr>("MotionPrimitive_W", this);
  uint T = birosInfo.getParameter<uint>("MotionPrimitive_TrajectoryLength", this);
  double duration = birosInfo.getParameter<double>("MotionPrimitive_TrajectoryDuration", this);
  
  //clone the geometric state
  s->geo.pull();
  /*geo->readAccess(this);
//   s->ors = geo->ors.newClone();
  geo->deAccess(this);*/
  if (s->verbose) {
    s->gl = new OpenGL("MotionPrimitive");
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

void MotionPrimitive::close() {
}

void MotionPrimitive::step() {
  s->geo.pull();
  
  MotionKeyframe *frame0 = plan->get_frame0(this);
  MotionKeyframe *frame1 = plan->get_frame1(this);

  Action::ActionPredicate actionSymbol = action->get_action(this);
  
  if (actionSymbol==Action::noAction) {
    frame1->writeAccess(this);
    frame1->x_estimate = frame0->get_x_estimate(this);
    frame1->duration_estimate = 0.;
    frame1->converged = true;
    frame1->deAccess(this);
    
    plan->writeAccess(this);
    plan->steps = 0;
    plan->tau = 0.;
    listDelete(plan->TVs);
    plan->converged=true;
    plan->q_plan.clear();
    plan->deAccess(this);
  }
  
  if (actionSymbol==Action::grasp || actionSymbol==Action::place) {
    
    if (frame1->get_converged(this)) {
      //action->waitForConditionSignal(.01);
      return;
    }
    
    //pull start condition
    arr x0;
    frame0->get_x_estimate(x0, this);
    CHECK(x0.N==2*s->sys.qDim(),"You need to initialize frame0 to start pose!");
    s->sys.setx0(x0);
    
    //estimate the key frame
    arr x_keyframe;
    if (actionSymbol==Action::grasp) {
      uint shapeId = s->sys.ors->getShapeByName(action->get_objectRef1(this))->index;
      threeStepGraspHeuristic(x_keyframe, s->sys, x0, shapeId, s->verbose);
    }
    else if (actionSymbol==Action::place) {
      s->sys.setx0(x0);
      listDelete(s->sys.vars);
      uint shapeId = s->sys.ors->getShapeByName(action->get_objectRef1(this))->index;
      uint fromId = s->sys.ors->getShapeByName(action->get_objectRef2(this))->index;
      uint toId = s->sys.ors->getShapeByName(action->get_objectRef3(this))->index;
      setPlaceGoals(s->sys,s->sys.nTime(),shapeId,fromId,toId);
      keyframeOptimizer(x_keyframe, s->sys, 1e-2, false, s->verbose);
    }
    else if (actionSymbol==Action::home) {
      s->sys.setx0(x0);
      listDelete(s->sys.vars);
      //setHomingGoals(sys,T,shapeId, side, 1);
      //keyframeOptimizer(x, s->sys, 1e-2, true, verbose);
    }

    //push it
    frame1->writeAccess(this);
    frame1->x_estimate = x_keyframe;
    frame1->duration_estimate = s->sys.getDuration();
    frame1->converged = true;
    frame1->deAccess(this);
    
    //-- start planner
    plan->writeAccess(this);
    //info on the plan: steps, duration, boundary
    plan->converged = false;
    plan->steps = s->sys.nTime();
    plan->tau = s->sys.getDuration() / plan->steps;
    //details on the task costs
    listClone(plan->TVs, s->sys.vars);
    plan->deAccess(this);
  }
  
  if (actionSymbol==Action::place) {
    //setPlaceGoals(*sys, sys->nTime(), goalVar->graspShape, goalVar->belowFromShape, goalVar->belowToShape);
    //arr q;
    //soc::straightTaskTrajectory(*sys, q, 0);
    //planner.init_trajectory(q);
  }
  
  if (actionSymbol==Action::home) {
    //setHomingGoals(*sys, sys->nTime(), goalVar->graspShape, goalVar->belowToShape);
    //arr q;
    //soc::straightTaskTrajectory(*sys, q, 1); //task id is q!!!
    //planner.init_trajectory(q);
  }
  
  //action->waitForConditionSignal(.01);
  //... to set task variables
  
  //FUTURE: collaps all the task variable stuff to a single Phi
};


//===========================================================================
//
// the rest is on the three base routines
//

void threeStepGraspHeuristic(arr& x, soc::SocSystem_Ors& sys, const arr& x0, uint shapeId, uint verbose) {
  uint T = sys.nTime();
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
  static bool firstTime=true;
  static double endEffPrec, endOppPrec, endAlignPrec, graspDistPrec, colPrec, limPrec, zeroQPrec;
  if (firstTime) {
    firstTime=false;
    endEffPrec = birosInfo.getParameter<double>("graspPlanEndEffPrec");
    endOppPrec = birosInfo.getParameter<double>("graspPlanEndOppPrec");
    endAlignPrec = birosInfo.getParameter<double>("graspPlanEndAlignPrec");
    graspDistPrec = birosInfo.getParameter<double>("graspPlanGraspDistPrec");
    colPrec = birosInfo.getParameter<double>("graspPlanColPrec");
    limPrec = birosInfo.getParameter<double>("graspPlanLimPrec");
    zeroQPrec = birosInfo.getParameter<double>("graspPlanHomePrec");
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
  V->y_prec = endEffPrec;
  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., 0.);
  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);
  
  //up: align either with cylinder axis or one of the box sides -- works good
  V=new DefaultTaskVariable("upAlign", *sys.ors, zalignTVT, "graspCenter", obj->name, arr());
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
  V->y_prec = endAlignPrec;
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
  V->y_prec = graspDistPrec;
  V->setInterpolatedTargetsEndPrecisions(T,colPrec,graspDistPrec,0.,0.);
  for (uint t=0; t<=T; t++) { //interpolation: 0 up to 4/5 of the trajectory, then interpolating in the last 1/5
    if (5*t<4*T) V->y_trajectory[t]()=0.;
    else V->y_trajectory[t]() = (grip*double(5*t-4*T))/T;
  }
  sys.vars.append(V);
  
  //collisions with other objects
  shapes = ARRAY(shapeId);
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
  V->y_prec=endOppPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., endOppPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);
  V = new DefaultTaskVariable("oppose13", *sys.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip3", "<d( 90 1 0 0)>", 0);
  V->y_target = ARR(-1.);  V->v_target = ARR(0.);
  V->y_prec=endOppPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., endOppPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
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

void setPlaceGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint belowFromShapeId, uint belowToShapeId){
  sys.setTox0();
  
  double placeMidPrec        = birosInfo.getParameter<double>("placeMidPrec");
  double placeAlignmentPrec  = birosInfo.getParameter<double>("placeAlignmentPrec");
  double limPrec             = birosInfo.getParameter<double>("limPrec");
  double zeroQPrec           = birosInfo.getParameter<double>("zeroQPrec");
  double placePositionPrec   = birosInfo.getParameter<double>("placePositionPrec");
  double placeUpDownVelocity = birosInfo.getParameter<double>("placeUpDownVelocity");
  double placeUpDownVelocityPrec = birosInfo.getParameter<double>("placeUpDownVelocityPrec");

  
  //set the time horizon
  CHECK(T==sys.nTime(), "");
  
  //deactivate all variables
  activateAll(sys.vars, false);
  
  //activate collision testing with target shape
  ors::Shape *obj  = sys.ors->shapes(shapeId);
  ors::Shape *from = sys.ors->shapes(belowFromShapeId);
  ors::Shape *onto = sys.ors->shapes(belowToShapeId);
  CHECK(obj->body==sys.ors->getBodyByName("m9"), "called planPlaceTrajectory without right object in hand");
  obj->cont=true;
  onto->cont=false;
  from->cont=false;
  sys.swift->initActivations(*sys.ors);
  
  TaskVariable *V;
  
  //general target
  arr xtarget;
  xtarget.setCarray(onto->X.pos.p, 3);
  xtarget(2) += .5*(onto->size[2]+obj->size[2])+.005; //above 'place' shape
  
  //endeff
  V = new DefaultTaskVariable("graspCenter", *sys.ors, posTVT, "graspCenter", NULL, NULL);
  ((DefaultTaskVariable*)V)->irel = obj->rel;
  V->updateState(*sys.ors);
  V->y_target = xtarget;
  V->setInterpolatedTargetsEndPrecisions(T, placeMidPrec, placePositionPrec, 0., 0.);
  //special: condition effector velocities:
  uint t, M=T/8;
  for(t=0; t<M; t++){
    V -> v_trajectory[t]() = (1./M*t)*ARR(0., 0., placeUpDownVelocity);
    V -> v_prec_trajectory(t) = placeUpDownVelocityPrec;
  }
  for(t=T-M; t<T; t++){
    V -> v_trajectory[t]() = (1./M*(T-t))*ARR(0., 0., -placeUpDownVelocity); //0.2
    V -> v_prec_trajectory(t) = placeUpDownVelocityPrec; // 1e1
  }
  sys.vars.append(V);
  
  //up1
  V = new DefaultTaskVariable("up1", *sys.ors, zalignTVT, "m9", "<d(90 1 0 0)>", 0, 0, 0);
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V) -> irel.addRelativeRotationDeg(90, 1, 0, 0);
  V->updateState(*sys.ors);
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, placeMidPrec, placeAlignmentPrec, 0., 0.);
  sys.vars.append(V);
  
  //up2
  V = new DefaultTaskVariable("up2", *sys.ors, zalignTVT, "m9", "<d( 0 1 0 0)>", 0, 0, 0);
  ((DefaultTaskVariable*)V)->irel = obj->rel;  ((DefaultTaskVariable*)V)-> irel.addRelativeRotationDeg(90, 0, 1, 0);
  V->updateState(*sys.ors);
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, placeMidPrec, placeAlignmentPrec, 0., 0.);
  sys.vars.append(V);
  
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

//void setHomingGoals(soc::SocSystem_Ors& sys, uint T, const char* objShape, const char* belowToShape){
  //sys.setx0ToCurrent();
  
  ////deactivate all variables
  //activateAll(sys.vars, false);
  
  //ors::Shape *obj  = sys.ors->getShapeByName(objShape);
  //ors::Shape *onto = sys.ors->getShapeByName(belowToShape);
  //obj->cont=true;
  //onto->cont=true;
  //sys.swift->initActivations(*sys.ors);
  
  //TaskVariable *V;
  
  ////general target
  //double midPrec, endPrec;
  //MT::getParameter(midPrec, "homingPlanMidPrec");
  //MT::getParameter(endPrec, "homingPlanEndPrec");
  
  ////endeff
  //V=listFindByName(sys.vars, "endeffector");
  ////V->irel = obj->rel;
  //V->updateState(*sys.ors);
  //V->setInterpolatedTargetsEndPrecisions(T, 0, 0, 0., 0.);
  ////special: condition effector velocities: move above object
  //uint t, M=T/8;
  //for(t=0; t<M; t++){
    //V -> v_trajectory[t]() = (1./M*t)*ARR(0., 0., .2);
    //V -> v_prec_trajectory(t) = 1e1;
  //}
  ////for(t=M;t<T;t++){
  ////  V -> v_trajectory[t]() = 0;
  ////  V -> v_prec_trajectory(t) = 0;
  ////}
  
  ////col lim and relax
  //V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanColPrec"), 0.);
  //V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, MT::getParameter<double>("reachPlanLimPrec"), 0.);
  //V=listFindByName(sys.vars, "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;
  //V->setInterpolatedTargetsEndPrecisions(T,
                                         //midPrec, endPrec,
                                         //midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
//}

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
    sys.getW(W,0);
    arr wdiag;
    getDiag(wdiag, W);
    wdiag *= double(sys.nTime());
    for (uint i=0; i<wdiag.N; i++) wdiag(i) = 1./sqrt(wdiag(i));
    sqrtWinv = diag(wdiag);
  } else {
    //From Dmitry
    double T = sys.nTime();
    arr H1,Q,Q1,Q2;
    sys.getHrateInv(H1);
    sys.getQrate(Q);
    
    decomposeMatrix(Q1,Q2,Q);
    double tau = sys.getDuration();// tau is basically = time
    double tau2=tau*tau;
    int dim=sqrt(Q.N)/2;
    
    arr I,Z,AT,Zv;
    I.setId(dim); Z.resize(dim,dim); Z.setZero();
    AT.setBlockMatrix(I,sys.getDuration()*I,Z,I);  // A to the power of T
    
    double S0 = SumOfRow(T,0); double S1 = SumOfRow(T-1,1); double S2 = SumOfRow(T-1,2);  // sums of geometric series
    arr sigma1,sigma2,sigma3,sigma4; // Blocks of sigma matrix
    sigma1 = tau2*tau*H1*(S0+2.0*S1 + S2)/pow(T,3) + tau2*tau*Q2*S2/pow(T,3)+ tau*S0*Q1/T;
    sigma2 = tau2*H1*(S0+S1)/pow(T,2) + tau2*S1*Q2/pow(T,2);
    sigma3 = sigma2;
    sigma4 = tau*S0*(H1 + Q2)/T;
    
    arr sumA,sumAinv;
    sumA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4);
    inverse_SymPosDef(sumAinv,sumA);
    //suma= AT*x0;
    lapack_cholesky(sqrtWinv, sumAinv);
  }
  
  sys.getx0(x0);
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
      sys->getTaskCostTerms(Phi, J, x, sys->nTime());
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

