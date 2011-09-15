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
  xtarget(2) += .02; //grasp it 2cm above center
  
  // graspCenter -> predefined point (xtarget)
  V = new DefaultTaskVariable("graspCenter", *sys.ors, posTVT, "graspCenter", NULL, NULL);
  V->y_target = xtarget;
  V->y_prec = 1e3;
  V->setInterpolatedTargetsEndPrecisions(4*T/5, midPrec, 0.);
  V->appendConstTargetsAndPrecs(T);
  sys.vars.append(V);
  
  //up -- good
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
    ARRAY<const char*>("tip1Shape", "target",
                       "tip2Shape", "target",
                       "tip3Shape", "target"), sys.ors->shapes);
  V = new ProxyTaskVariable("graspContacts", *sys.ors, vectorCTVT, shapes, .04, true);
  double grip=.9;
  V->y_target = ARR(grip,grip,grip);  V->v_target = ARR(.0,.0,.0);
  V->y_prec = colPrec;
  V->setInterpolatedTargetsEndPrecisions(T,colPrec,1e1,0.,0.);
  for(uint t=0;t<=T;t++){
    if(5*t<4*T) V->y_trajectory[t]()=0.;
    else V->y_trajectory[t]() = (grip*double(5*t-4*T))/T;
  }
  sys.vars.append(V);
  
  //opposing fingers
  V=listFindByName(sys.vars, "oppose12");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, midPrec, endPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
  V=listFindByName(sys.vars, "oppose13");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(4*T/5, midPrec, endPrec, 0., 0.);  V->appendConstTargetsAndPrecs(T);
  
  //col lim and relax
  V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  V=listFindByName(sys.vars, "qitself");
  V->y_prec=MT::getParameter<double>("reachPlanHomeComfort");
  V->v_prec=MT::getParameter<double>("reachPlanEndVelPrec");
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, V->y_prec, V->y_prec, midPrec, V->v_prec);
}
