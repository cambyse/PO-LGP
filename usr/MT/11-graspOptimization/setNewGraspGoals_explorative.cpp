void setNewGraspGoals_explore(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase){
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
  
  //-- create a grasp object for the ors shape
  GraspObject *graspobj;
  switch(obj->type){
      //graspobj = new GraspObject_InfCylinder(ARRAY(obj->X.pos), ARR(0,0,1), .04, 1.);
    case ors::cylinderST:  graspobj = new GraspObject_Cylinder1(obj);  break;
    case ors::boxST:  graspobj = new GraspObject_Box(obj);  break;
    default: NIY;
  }
  graspobj->distanceMode = true;
#if 0
  graspobj->buildMesh();
  graspobj->saveMesh("grasp_mesh.tri");
#else
  //graspobj->loadMesh("grasp_mesh.tri");
#endif
  //sys.gl->add(glDrawMeshObject, graspobj);

  TaskVariable *V;

  // graspCenter -- really necessary? yes, clean up
#if 1 // graspCenter -> predefined point (xtarget)
  //general target
  arr xtarget(obj->X.pos.p, 3);
  xtarget(2) += .02; //grasp it 2cm above center
  
  //endeff
  V = new DefaultTaskVariable("graspCenter", *sys.ors, posTVT, "graspCenter", NULL, NULL);
  V->y_target = xtarget;
  V->y_prec = 1e3;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, 0.);
  sys.vars.append(V);
#else // graspCenter -> negative level
  //MT: somehow this doesn't work: it seems the gradients inside are just too messy!
  V = new PotentialValuesTaskVariable("graspCenterInsideLevel", *sys.ors, ARRAY(sys.ors->getShapeByName("graspCenter")), *graspobj);
  V->y_target = ARR(-.01);
  V->y_prec = 1e4;
  V->setInterpolatedTargetsEndPrecisions(T,0.,0.);
  sys.vars.append(V);
#endif
  
  //up -- good
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

  if(phase==0) return;
  
  //finger tips
  MT::Array<ors::Shape*> tipsN;
  tipsN.append(sys.ors->getShapeByName("tipNormal1"));
  tipsN.append(sys.ors->getShapeByName("tipNormal2"));
  tipsN.append(sys.ors->getShapeByName("tipNormal3"));

  //CLEAN away
  MT::Array<ors::Shape*> hooksN;
  hooksN.append(sys.ors->getShapeByName("tipHook1"));
  hooksN.append(sys.ors->getShapeByName("tipHook2"));
  hooksN.append(sys.ors->getShapeByName("tipHook3"));
  /* inside gradients just don't work :-(
  V = new PotentialValuesTaskVariable("hooksInsideLevel", *sys.ors, hooksN, *graspobj);
  //V=listGetByName(sys.vars,"zeroLevel");
  V->y_target = ARR(-.01,-.01,-.01);
  V->y_prec = 1e4;
  V->setInterpolatedTargetsEndPrecisions(T,0.,0.);
  //sys.vars.append(V); */

  //replaced by proxyTV
  V = new PotentialValuesTaskVariable("tipsOnZeroLevel", *sys.ors, tipsN, *graspobj);
  V->y_target = ARR(.005,.005,.005); 
  V->v_target = ARR(-1.,-1.,-1.); 
  V->y_prec = 1e3;
  V->setInterpolatedTargetsEndPrecisions(T,1e2,0.);
  //for(uint t=0;t<T;t++) V->y_trajectory[t]()=.2;  V->y_trajectory[T]=V->y_target;
  //V->v_trajectory.setZero();  V->v_trajectory[T]=V->v_target;
  //sys.vars.append(V);

  /* not really necessary
  V = new PotentialFieldAlignTaskVariable("tips z align", *sys.ors, tipsN, *graspobj);
  V->y_target = ARR(-1.,-1.,-1.); 
  V->y_prec = 1e2;
  V->setInterpolatedTargetsEndPrecisions(T,0.,0.);
  sys.vars.append(V);
  */

  uintA shapes = stringListToShapeIndices(
    ARRAY<const char*>("tip1Shape", "target",
                       "tip2Shape", "target",
                       "tip3Shape", "target"), sys.ors->shapes);
  V = new ProxyTaskVariable("graspContacts", *sys.ors, vectorCTVT, shapes, .04, true);
  V->y_target = ARR(.9,.9,.9);  V->v_target = ARR(.9,.9,.9);
  V->y_prec = colPrec;
  V->setInterpolatedTargetsEndPrecisions(T,colPrec,1e1,0.,0.);
  sys.vars.append(V);
  
  //opposing fingers
  V=listFindByName(sys.vars, "oppose12");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  V=listFindByName(sys.vars, "oppose13");  V->y_prec=endPrec;  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  
  //col lim and relax
  //V=listFindByName(sys.vars, "collision");  V->y=0.;  V->y_target=0.;  V->y_prec=colPrec;  V->setInterpolatedTargetsConstPrecisions(T);
  //V=listFindByName(sys.vars, "limits");     V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setInterpolatedTargetsConstPrecisions(T);
  V=listFindByName(sys.vars, "qitself");
  V->y_prec=MT::getParameter<double>("reachPlanHomeComfort");
  V->v_prec=MT::getParameter<double>("reachPlanEndVelPrec");
  //V->y_target = ARR(0,0,0,0,0,0,0);
  //V->y_target.append(ARR(0,-.8,.6,-.8,.6,-.8,.6));
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setInterpolatedTargetsEndPrecisions(T, V->y_prec, V->y_prec, midPrec, V->v_prec);
}
