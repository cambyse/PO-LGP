#include "setGraspGoals.h"
#include "opengl.h"

void threeStepGraspHeuristic(soc::SocSystem_Ors& sys, arr& q, const arr& q0, uint shapeId, uint verbose){
  uint T = sys.nTime();
  //double duration = sys.getTau() * T;
  
  sys.setq0(q0);
  listDelete(sys.vars);

  uint side=0;

  //-- optimize ignoring hand -- testing different options for aligning with the object
  if(sys.ors->shapes(shapeId)->type==ors::boxST){
    arr cost_side(3),q_side(3,q0.N);
    for(side=0;side<3;side++){
      setGraspGoals(sys, T, shapeId, side, 0);
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
    setGraspGoals(sys, T, shapeId, side, 0);
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
  setGraspGoals(sys,T,shapeId, side, 1);
  KeyframeOptimizer(q, sys, 1e-2, true, verbose);
  //listDelete(sys.vars); //DON'T delete the grasp goals - the system should keep them for the planner
  if(verbose>=1) sys.displayState(NULL, NULL, "posture estimate phase 2", false);
  if(verbose>=2) sys.gl->watch();
}

void setGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase){
  sys.setTox0();
  
  //load parameters only once!
  static bool firstTime=true;
  static double endEffPrec, endOppPrec, endAlignPrec, graspDistPrec, colPrec, limPrec, homePrec;
  if(firstTime){
    firstTime=false;
    MT::getParameter(endEffPrec, "graspPlanEndEffPrec");
    MT::getParameter(endOppPrec, "graspPlanEndOppPrec");
    MT::getParameter(endAlignPrec, "graspPlanEndAlignPrec");
    MT::getParameter(graspDistPrec, "graspPlanGraspDistPrec");
    MT::getParameter(colPrec, "graspPlanColPrec");
    MT::getParameter(limPrec, "graspPlanLimPrec");
    MT::getParameter(homePrec, "graspPlanHomePrec");
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
  V->y_prec = endAlignPrec;
  //V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  V->setInterpolatedTargetsEndPrecisions(4*T/5, 0., 0.);
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
  double grip=.8; //specifies the desired proxy value
  V->y_target = ARR(grip,grip,grip);  V->v_target = ARR(.0,.0,.0);
  V->y_prec = graspDistPrec;
  V->setInterpolatedTargetsEndPrecisions(T,colPrec,graspDistPrec,0.,0.);
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
  V->y_prec=homePrec;
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setConstTargetsConstPrecisions(T);
  sys.vars.append(V);
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

