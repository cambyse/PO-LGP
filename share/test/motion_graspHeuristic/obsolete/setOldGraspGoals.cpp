void setOldGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase){
  sys.setx0ToCurrent();
  
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
  CHECK(T==sys.get_T(), "");
  
  //deactivate all variables
  activateAll(sys.vars, false);
  
  //activate collision testing with target shape
  ors::Shape *obj = sys.ors->shapes(shapeId);
  obj->cont=true;
  sys.swift->initActivations(*sys.ors);
  
  TaskVariable *V;
  
  //general target
  arr xtarget;
  xtarget.setCarray(obj->X.pos.p, 3);
  xtarget(2) += .02; //grasp it 2cm above center
  
  //endeff
  V=listFindByName(sys.vars, "endeffector");
  ((DefaultTaskVariable*)V)->irel.setText("<t(0 0 -.26)>");
  V->updateState();
  V->y_target = xtarget;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, palmPrec, 0., 0.);
  
  //up
  V=new DefaultTaskVariable("upAlign", *sys.ors, zalignTVT, "graspCenter", obj->name, arr());
  ((DefaultTaskVariable*)V)->irel.setText("<d(90 1 0 0)>");
  switch(obj->type){
    case ors::cylinderST:
      V->y_target = 0.;  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case ors::boxST:{
      /*rnd.clockSeed();
      static int side=-1;
      if(side==-1) side=rnd(3);
      cout <<"*** side = " <<side <<endl;*/
      ((DefaultTaskVariable*)V)->jrel=obj->X;
      if(side==1) ((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,1,0,0);
      if(side==2) ((DefaultTaskVariable*)V)->jrel.addRelativeRotationDeg(90,0,1,0);
      V->y_target = 1.;  //y-axis of m9 is aligned with one of the 3 sides of the cube
    }break;
    default: NIY;
  }
  //cout <<V->irel <<V->jrel <<endl;
  V->updateState();
  if(V->y(0)<0.) ((DefaultTaskVariable*)V)->irel.addRelativeRotationDeg(180,1,0,0); //flip vector to become positive
  V->updateState();
  V->y_prec = 1e3; //endPrec;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  sys.vars.append(V);
  
  //finger tip hooks
  //V=listFindByName(sys.vars, "pos1");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  //V=listFindByName(sys.vars, "pos2");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  //V=listFindByName(sys.vars, "pos3");  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);

  double radius = .05;
  V=new DefaultTaskVariable("hook1", *sys.ors, posTVT, "tipNormal1", NULL, ARR());
  ((DefaultTaskVariable*)V)->irel.addRelativeTranslation(0,0,radius);
  V->updateState();
  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  sys.vars.append(V);

  V=new DefaultTaskVariable("hook2", *sys.ors, posTVT, "tipNormal2", NULL, ARR());
  ((DefaultTaskVariable*)V)->irel.addRelativeTranslation( .033,0,radius);
  V->updateState();
  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  sys.vars.append(V);

  V=new DefaultTaskVariable("hook3", *sys.ors, posTVT, "tipNormal3", NULL, ARR());
  ((DefaultTaskVariable*)V)->irel.addRelativeTranslation(-.033,0,radius);
  V->updateState();
  V->y_target = xtarget;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  sys.vars.append(V);

  
  //opposing fingers
  V=listFindByName(sys.vars, "oppose12");  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "oppose13");  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  uintA shapes;  ors::Shape *s;
  s = listFindByName(sys.ors->shapes, "tip1Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "tip2Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "tip3Shape");  shapes.append(s->index);
  s = listFindByName(sys.ors->shapes, "target");  shapes.append(s->index);
  //shapes.append(shapeId);
  V = new ProxyTaskVariable("graspContacts", *sys.ors, vectorCTVT, shapes, .04, true);
  V->y_target = ARR(.0,.0,.0);  V->v_target = ARR(.0,.0,.0);
  V->y_prec = colPrec;
  V->setInterpolatedTargetsConstPrecisions(T,colPrec,0.);
  sys.vars.append(V);
  //V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
  //V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->setInterpolatedTargetsConstPrecisions(T, limPrec, 0.);
  V=listFindByName(sys.vars, "qitself");    V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, MT::getParameter<double>("reachPlanHomeComfort"), 0., midPrec, MT::getParameter<double>("reachPlanEndVelPrec"));
}
