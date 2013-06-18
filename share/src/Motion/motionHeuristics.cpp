#include "motionHeuristics.h"
#include "taskMap_default.h"
#include "taskMap_proxy.h"
#include <Optim/optimization.h>

#include <Gui/opengl.h>


//===========================================================================
//
// the rest is on the three base routines
//

void threeStepGraspHeuristic(arr& x, MotionProblem& M, const arr& x0, uint shapeId, uint verbose) {
  OpenGL gl;
  bindOrsToOpenGL(*M.ors, gl);

  uint T = M.T;
  //double duration = sys.getTau() * T;
  
  M.setx0(x0);
  listDelete(M.taskCosts());
  
  uint side=0;
  
  //-- optimize ignoring hand -- testing different options for aligning with the object
  if (M.ors->shapes(shapeId)->type==ors::boxST) {
    arr cost_side(3),x_side(3,x0.N);
    for (side=0; side<3; side++) {
      setGraspGoals(M, T, shapeId, side, 0);
      cost_side(side) = keyframeOptimizer(x, M, 1e-2, false, verbose);
      listDelete(M.taskCosts());
      if (verbose>=2) {
        displayState(x, *M.ors, gl, "posture estimate phase 0");
//        M.displayCurrentState("posture estimate phase 0", false, false);
        //M.gl->watch();
      }
      x_side[side]() = x;
    }
    cout <<"3 side costs=" <<cost_side <<endl;
    side = cost_side.minIndex();
    x = x_side[side];
  } else {
    setGraspGoals(M, T, shapeId, side, 0);
    keyframeOptimizer(x, M, 1e-2, false, verbose);
    listDelete(M.taskCosts());
    if (verbose>=2) {
      displayState(x, *M.ors, gl, "posture estimate phase 0");
//      M.displayCurrentState("posture estimate phase 0", false, false);
     // M.gl->watch();
    }
  }
  
  //-- open hand
  x.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
//  M.setx(x);
  if (verbose>=2) {
    displayState(x, *M.ors, gl, "posture estimate phase 1");
//    M.displayCurrentState("posture estimate phase 1", false, false);
    //M.gl->watch();
  }
  
  //-- reoptimize with close hand
  setGraspGoals(M,T,shapeId, side, 1);
  keyframeOptimizer(x, M, 1e-2, true, verbose);
  //listDelete(M.vars); //DON'T delete the grasp goals - the system should keep them for the planner
  if (verbose>=1) displayState(x, *M.ors, gl, "posture estimate phase 2");
//  M.displayCurrentState("posture estimate phase 2", false, false);
  //if (verbose>=2) M.gl->watch();
}

void setGraspGoals(MotionProblem& M, uint T, uint shapeId, uint side, uint phase) {
  M.setState(M.x0, M.v0);;
  
  //load parameters only once!
  double positionPrec = MT::getParameter<double>("graspPlanPositionPrec");
  double oppositionPrec = MT::getParameter<double>("graspPlanOppositionPrec");
  double alignmentPrec = MT::getParameter<double>("graspPlanAlignmentPrec");
  double fingerDistPrec = MT::getParameter<double>("graspPlanFingerDistPrec");
  double colPrec = MT::getParameter<double>("graspPlanColPrec");
  double limPrec = MT::getParameter<double>("graspPlanLimPrec");
  double zeroQPrec = MT::getParameter<double>("graspPlanZeroQPrec");
  
  //set the time horizon
  CHECK(T==M.T, "");
  
  //deactivate all variables
  M.activateAllTaskCosts(false);
  
  //activate collision testing with target shape
  ors::Shape *target_shape = M.ors->shapes(shapeId);
  target_shape->cont=true;
  M.swift->initActivations(*M.ors);
  
  //
  arr target,initial;
  ors::Transformation irel,jrel;

  //general target
  target = ARRAY(target_shape->X.pos);
  //xtarget(2) += .02; //grasp it 2cm above center
  
  // graspCenter -> predefined point (xtarget)
//  V = new DefaultTaskVariable("graspCenter", *M.ors, posTVT, "graspCenter", NULL, NoArr);
//  V->y_target = xtarget;
//  V->y_prec = positionPrec;
//  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., 0.);
//  V->appendConstTargetsAndPrecs(T);
//  M.vars().append(V);
  TaskCost *c;
  c = M.addDefaultTaskMap_Shapes("graspCenter", posTMT, "graspCenter");
  M.setInterpolatingCosts(c, MotionProblem::constEarlyMid,
                          target, positionPrec, NoArr, -1., .8);

  
  //up: align either with cylinder axis or one of the box sides -- works good
  irel.setText("<d(90 1 0 0)>");
  jrel.setZero();
  switch (target_shape->type) {
    case ors::cylinderST:
      target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case ors::boxST: {
      jrel=target_shape->X;
      if (side==1) jrel.addRelativeRotationDeg(90,1,0,0);
      if (side==2) jrel.addRelativeRotationDeg(90,0,1,0);
      target = 1.;  //y-axis of m9 is aligned with one of the 3 sides of the cube
    } break;
    default: NIY;
  }
  c = M.addDefaultTaskMap_Shapes("upAlign", zalignTMT, "graspCenter", irel, target_shape->name, jrel, NoArr);
  c->map.phi(initial, NoArr, *M.ors);
//  V=new DefaultTaskVariable("upAlign", *M.ors, zalignTVT, "graspCenter", obj->name, NoArr);
//  V->updateState(*M.ors);

//  if (V->y(0)<0.)((DefaultTaskVariable*)V)->irel.addRelativeRotationDeg(180,1,0,0); //flip vector to become positive
  if (initial(0)<0.) ((DefaultTaskMap*)&c->map)->irel.addRelativeRotationDeg(180,1,0,0); //flip vector to become positive

//  V->updateState(*M.ors);
//  V->y_prec = alignmentPrec;
//  //V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
//  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., 0.);
//  V->appendConstTargetsAndPrecs(T);
//  M.vars().append(V);
  M.setInterpolatingCosts(c, MotionProblem::constEarlyMid,
                          target, alignmentPrec, NoArr, -1., .8);

  if (phase==0) return;
  
  //finger tips close to surface : using ProxyTaskVariable
  uintA shapes = stringListToShapeIndices(
                   ARRAY<const char*>("tip1Shape",
                                      "tip2Shape",
                                      "tip3Shape"), M.ors->shapes);
  shapes.append(shapeId); shapes.append(shapeId); shapes.append(shapeId);
  shapes.reshape(2,3); shapes = ~shapes;
  //V = new ProxyTaskVariable("graspContacts", *M.ors, vectorPTMT, shapes, .05, true);
  c = M.addCustomTaskMap("graspContacts", new ProxyTaskMap(vectorPTMT, shapes, .05, true));
  double grip=.8; //specifies the desired proxy value
  target = ARR(grip,grip,grip);
  M.setInterpolatingCosts(c, MotionProblem::constEarlyMid,
                          target, fingerDistPrec);
  //V->setInterpolatedTargetsEndPrecisions(T,colPrec,fingerDistPrec,0.,0.);
  for (uint t=0; t<=T; t++) { //interpolation: 0 up to 4/5 of the trajectory, then interpolating in the last 1/5
    if (5*t<4*T) c->y_target[t]()=0.;
    else c->y_target[t]() = (grip*double(5*t-4*T))/T;
  }
  //M.vars().append(V);
  
  //collisions with other objects
  shapes = ARRAY<uint>(shapeId);
  //V = new ProxyTaskVariable("otherCollisions", *M.ors, allExceptListedPTMT, shapes, .04, true);
  c = M.addCustomTaskMap("otherCollisions", new ProxyTaskMap(allExceptListedPTMT, shapes, .04, true));
  target = ARR(0.);
  M.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          target, colPrec);
  c->map.phi(initial, NoArr, *M.ors);
  if (initial(0)>0.) { //we are in collision/proximity -> depart slowly
    double a=initial(0);
    for (uint t=0; t<=T/5; t++)
      c->y_target[t]() = a*double(T-5*t)/T;
  }
  
  //opposing fingers
  //V = new DefaultTaskVariable("oppose12", *M.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip2", "<d( 90 1 0 0)>", 0);
  c = M.addDefaultTaskMap_Bodies("oppose12", zalignTMT,
                                 "tip1", ors::Transformation().setText("<d(90 1 0 0)>"),
                                 "tip2",  ors::Transformation().setText("<d( 90 1 0 0)>"));
  target = ARR(-1.);
  M.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          target, oppositionPrec);
  //V->y_prec=oppositionPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., oppositionPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
  //M.vars().append(V);


  //V = new DefaultTaskVariable("oppose13", *M.ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip3", "<d( 90 1 0 0)>", 0);
  c = M.addDefaultTaskMap_Bodies("oppose13", zalignTMT,
                                 "tip1",  ors::Transformation().setText("<d(90 1 0 0)>"),
                                 "tip3",  ors::Transformation().setText("<d( 90 1 0 0)>"));
  target = ARR(-1.);
  M.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          target, oppositionPrec);
  //V->y_prec=oppositionPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., oppositionPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
  //M.vars().append(V);
  
  //MT_MSG("TODO: fingers should be in relaxed position, or aligned with surface (otherwise they remain ``hooked'' as in previous posture)");
  
  //col lim and relax
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  //TODO: limits as parameter!
  //V = new DefaultTaskVariable("limits", *M.ors, qLimitsTVT, 0, 0, 0, 0, limits);
  c = M.addDefaultTaskMap("limits", qLimitsTMT);
  target=0.;  //V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  M.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          target, limPrec, target, limPrec);

  //V = new DefaultTaskVariable("qitself", *M.ors, qItselfTVT, 0, 0, 0, 0, 0);
  c = M.addDefaultTaskMap("qitself", qItselfTMT);
  M.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          target, zeroQPrec, target, zeroQPrec);
//  V->y_prec=zeroQPrec;
//  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setConstTargetsConstPrecisions(T);
//  M.vars().append(V);
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


double keyframeOptimizer(arr& x, MotionProblem& M, double stopTolerance, bool x_is_initialized, uint verbose) {
  arr sqrtWinv,x0;
  
  if (M.transitionType==MotionProblem::kinematic) {
    arr wdiag = M.H_rate_diag;
    wdiag *= double(M.T);
    for (uint i=0; i<wdiag.N; i++) wdiag(i) = 1./sqrt(wdiag(i));
    sqrtWinv = diag(wdiag);
  } else {
    NIY;
#if 0
    //From Dmitry
    double T = M.T;
    //control costs
    arr HrateInv;
    HrateInv.setDiag(1./M.H_rate_diag);

    //dynamics noise
    arr A,a,B,Q,Qrate,Q1,Q2;
    M.getDynamics(A,a,B,Q,0);
    Qrate = Q/M.get_tau();
    decomposeMatrix(Q1,Q2,Q);

    double tau = M.T*M.get_tau();// tau is basically = time
    double tau2=tau*tau;
    int dim=sqrt(Q.N)/2;
    
    arr I,Z,AT,Zv;
    I.setId(dim); Z.resize(dim,dim); Z.setZero();
    AT.setBlockMatrix(I,tau*I,Z,I);  // A to the power of T
    
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
#endif
  }
  
  x0 = M.x0;
  if (!x_is_initialized) x=x0;
  
  struct MyOptimizationProblem:VectorFunction {
    MotionProblem *M;
    arr sqrtWinv,x0;
    bool verbose;

    void   fv(arr& Phi, arr& J, const arr& x) {
      M->setState(x, NoArr);
//      if (verbose) {
//        sys->displayCurrentState("posture", false, true);
//        sys->gl->watch();
//      }
      //P.getTaskCosts(_phi, J_x, J_v, t);
      M->getTaskCosts(Phi, J, NoArr, M->T);
      Phi.append(sqrtWinv*(x-x0));
      if (&J) J.append(sqrtWinv);
    }
  } F;
  F.M = &M;
  F.sqrtWinv = sqrtWinv;
  F.x0=x0;
  F.verbose=false;
  if (verbose>=3) checkJacobian(F, x, 1e-6);
  F.verbose = verbose>=3;
  
  double cost;
  OptOptions opt;
  opt.fmin_return=&cost;
  opt.stopTolerance=1e-2;
  opt.stopEvals=100;
  opt.maxStep=.5;
  opt.damping=1.;
  opt.useAdaptiveDamping=true;
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
