//
// C++ Interface: NikolayGraspRoutines
//
// Description: 
//
//
// Author: Nikolay Jetchev,,,, <nikolay@nikolay>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

using namespace soc;

arr computeTotalCost( soc::SocSystem_Ors  & top,const arr& q, bool bSlow){
  uint t,T=top.nTime();
  CHECK(q.nd==2 && q.d0==T && q.d1==top.qDim(),"");
  double length=.0, cost1=0., cost2=0.;
  double cost_t, cost_s;
  arr W;
  top.getW(W,0);
  for(t=0;t<T;t++){
    top.setq(q[t]);
        //cout << endl << t << endl;
    if(bSlow&& false)top.s->ors->reportProxies();
    if(bSlow && useDisplay) cout << norm(top.s->vars(0)->x -top.s->vars(0)->x_trajectory[t]) << " and coll " <<norm(top.s->vars(1)->x ) <<   " and limit " <<norm(top.s->vars(2)->x -top.s->vars(2)->x_trajectory[t]);
    cost1 += cost_t = getTaskCost(top,t);  
    if(bSlow && useDisplay)  cout <<" cost for " << t << " = " <<cost_t <<endl;
    if(t>0) {cost_s = sqrDistance(W,q[t-1],q[t]);cost2 += cost_s;}
    if(t>0) length += metricDistance(W,q[t-1],q[t]);
      //   if(t >0){timeC(t-1+offset,0) = cost_t; timeC(t-1+offset,1) = cost_s;timeC(t-1+offset,2) = cost_s+ cost_t;} 
    if(useDisplay){
      top.gl->update();
      top.gl->text.clear() <<t << " (time " <<std::setw(3) <<t <<'/' <<T <<')';
      if(bSlow)top.gl->watch();
    }
  }
  arr ans(3);
  ans(0) = cost1;
  ans(1) = cost2;
  ans(2) = cost1 + cost2;
  cout << endl << cost1  << "|||"<< cost2 << "|||"<<cost2 + cost1 <<endl;
  return ans;
}
 
void PlanGrasp(OrsSocImplementation & soci){
  int display = useDisplay;
  OpenGL * gl = soci.gl;
    //BayesianKinematicMotionPlanning(soci,q,30,.7,.001,1);
  soc::AICO aico;
 
  double eps=MT::getParameter<double>("eps");
  double rate=MT::getParameter<double>("rate");
  uint K=MT::getParameter<uint>("K");
  
  for(uint k=0;k<K;k++){
    //col->setPrecisionTrajectoryConstant(T,k*colPrec);
    if(!soci.dynamic) aico.stepKinematic(soci,2,.7,eps,display);
    else{
      //if(k>5){ eps=0; rate=.5; }
      if(k<K/2) aico.stepDynamic(soci,2,rate,eps,display);
      else      iLQG_general(soci,aico.q,10,rate,display);
    }
  }
 // ofstream fil("z.plan");
 // aico.v.writeTagged(fil,"v");
//  aico.invV.writeTagged(fil,"invV");
  gl->watch();  
  soci.displayTrajectory(aico.q,1);
  gl->watch();  
}

void SetRelTarget(OrsSocImplementation & soci){
  int T = 300;
  arr target;
  target.setCarray(soci.s->ors->getName("target")->X.p.v,3);
 
  ControlVariable *CV_x  = soci.s->vars(0);
  ControlVariable *p1  = soci.s->vars(2);
  ControlVariable *p2  = soci.s->vars(3);
  ControlVariable *p3  = soci.s->vars(4);
      
  double midPrec,endPrec,balPrec,colPrec;
  MT::getParameter(midPrec,"midPrec");
  MT::getParameter(endPrec,"endPrec");
  MT::getParameter(balPrec,"balPrec");
  MT::getParameter(colPrec,"colPrec");
   
  p1->x_target = target;
  p2->x_target = target;
  p3->x_target = target;
  CV_x->x_target = target;
  
  CV_x->setInterpolatedTargetTrajectory(T);
  CV_x->setPrecisionTrajectoryFinal(T,midPrec,MT::getParameter<double>("palmPrec"));
  CV_x->setPrecisionVTrajectoryConstant(T,0.);

  p1->setInterpolatedTargetTrajectory(T);
  p1->setPrecisionTrajectoryFinal(T,midPrec,endPrec);
  p1->setPrecisionVTrajectoryConstant(T,0.);
  
  p2->setInterpolatedTargetTrajectory(T);
  p2->setPrecisionTrajectoryFinal(T,midPrec,endPrec);
  p2->setPrecisionVTrajectoryConstant(T,0.);
  
  p3->setInterpolatedTargetTrajectory(T);
  p3->setPrecisionTrajectoryFinal(T,midPrec,endPrec);
  p3->setPrecisionVTrajectoryConstant(T,0.);
  
  
}

void InitSOC(OrsSocImplementation & soci, ors::KinematicWorld & ors, SwiftInterface & swift, OpenGL & gl){
  uint T=300;
  arr W;
  W <<"[.1 .1 .2 .2 1 1 5    1 1 1 1 1 1 1 1 1]";
  if(!MT::getParameter<bool>("dynamic")){
    soci.initKinematic(&ors,&swift,&gl,T,&W);
  }else{
    soci.initPseudoDynamic(&ors,&swift,&gl,2.,T,&W);
  }
  soci.os=&cout;
 // gl.watch();

  ControlVariable *CV_x,*CV_rot,*CV_q,*CV_up,*CV_z1,*CV_z2,*CV_f1,*CV_f2,*CV_f3,*CV_col,*CV_lim;
  
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -2. 2.; -2. 2.; \
      -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0 ]";
  arr I; I.setId(ors.getJointStateDimension());
  //arr skinIdx; //copy(skinIdx,skinIndex);
  
  CV_x    = new ControlVariable("endeffector",ors, posCVT,"m9","<t(0 0 -.23)>",0,0,0);
  CV_rot  = new ControlVariable("endeffector rotation",ors, rotCVT,"m9",0,0,0,0);
  CV_col  = new ControlVariable("collision", ors, collCVT,0,0,0,0,ARR(.03)); //MARGIN, perhaps .05?
  CV_lim  = new ControlVariable("limits", ors, qLimitsCVT,0,0,0,0,limits);
  CV_q    = new ControlVariable("qlinear", ors, qLinearCVT,0,0,0,0,I);
  //CV_skin = new ControlVariable("skin", ors, skinCVT,0,0,0,0,skinIdx);
  CV_up   = new ControlVariable("endortho",ors, zalignCVT,"m9","<d(90 1 0 0)>",0,0,0);
  CV_z1   = new ControlVariable("align1",ors,zalignCVT,"tip1","<d(90 1 0 0)>","tip2","<d( 90 1 0 0)>",0);
  CV_z2   = new ControlVariable("align1",ors,zalignCVT,"tip1","<d(90 1 0 0)>","tip3","<d( 90 1 0 0)>",0);
  CV_f1   = new ControlVariable("pos1",ors,posCVT,"tip1","<t( .0   -.1 .0)>",0,0,0);
  CV_f2   = new ControlVariable("pos2",ors,posCVT,"tip2","<t( .033 -.1 .0)>",0,0,0);
  CV_f3   = new ControlVariable("pos3",ors,posCVT,"tip3","<t(-.033 -.1 .0)>",0,0,0);

  ControlVariableList CVall;
  CVall.append(TUPLE(CV_x,CV_rot,CV_col,CV_lim,CV_q));
  CVall.append(TUPLE(CV_up,CV_z1,CV_z2,CV_f1,CV_f2,CV_f3));
  soci.setControlVariables(CVall);

  //deactivate a couple of variables
  CV_q->active=false;
  CV_rot->active=false;
  //CV_skin->active=false;

  //activate collision testing with target shape
  ors.getShapeByName("targetshape")->cont=true;
  swift.initActivations(ors);
      
  //get target position
  arr target;
  target.setCarray(ors.getName("target")->X.p.v,3);
  CV_f1->x_target = target;
  CV_f2->x_target = target;
  CV_f3->x_target = target;
  CV_x ->x_target = target;
  CV_up ->x_target = 0.;
  CV_col->x = 0.;  CV_col->x_target = 0.;
  CV_lim->x = 0.;  CV_lim->x_target = 0.;

  //set precisions
  double midPrec,endPrec;
  MT::getParameter(midPrec,"reachPlanMidPrec");
  MT::getParameter(endPrec,"reachPlanEndPrec");
  CV_x  ->setInterpolatedTargetsEndPrecisions(T,midPrec,MT::getParameter<double>("reachPlanPalmPrec"),0.,0.);
  CV_up ->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  CV_f1 ->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  CV_f2 ->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  CV_f3 ->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  CV_z1 ->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  CV_z2 ->setInterpolatedTargetsEndPrecisions(T,midPrec,endPrec,0.,0.);
  CV_col->setInterpolatedTargetsConstPrecisions(T,MT::getParameter<double>("reachPlanColPrec"),0.);
  CV_lim->setInterpolatedTargetsConstPrecisions(T,MT::getParameter<double>("reachPlanLimPrec"),0.);

  //for(int i = soci.s->vars.N-2; i < soci.s->vars.N-1; i++)
  //  for(int t = 0; t < T; t++)
  //  cout << soci.s->vars(i)->x_trajectory[t];
  
  //CV_x->active=false;
  //CV_>up->active=false;
  //p1->active=false;
  //p2->active=false;
  //p3->active=false;
  //z1->active=false;
  //z2->active=false;
  //col->active=false;
  //CV_lim->active=false; 
}

void PlanGraspM( ors::KinematicWorld & ors, SwiftInterface & swift, OpenGL & gl){
  uint nTry=MT::getParameter<uint>("nTry");
  uint nStartTry=MT::getParameter<uint>("nStartTry");
  double eps=MT::getParameter<double>("reachPlanEps");
  double rate=MT::getParameter<double>("reachPlanRate");
  uint K=MT::getParameter<uint>("reachPlanK");
  ofstream f1(String("graspcost.txt") + nShape);
  ors::Body * target =  ors.getName("target");
  ors::Vector orig = target->X.p;
  arr q; ors.getJointState(q);
  for(uint i = nStartTry; i < nTry; i++){//different trials
    cout << "starting trial " << i << endl;
    rnd.seed(i*100);
    target->X.p(0) = orig(0) + (rnd.uni()-0.5)*0.5;
    target->X.p(1) = orig(1) + rnd.uni()*0.2-0.2;
    target->X.p(2) = orig(2) + (rnd.uni()-0.5)*0.35;
    ors.setJointState(q);
    ors.calcNodeFramesFromEdges();
    OrsSocImplementation  soci;
    InitSOC(soci,ors,swift,gl); 
    soc::AICO aico;
    
    for(uint k=0;k<K;k++){
      if(!soci.dynamic) aico.stepKinematic(soci,2,.7,eps,useDisplay);
      else{
        if(k<K/2 && true)
          aico.stepDynamic(soci,2,rate,eps,useDisplay);
        else     
          iLQG_general(soci,aico.q,10,rate,useDisplay);
      }
    }

    if(K){   
      f1 << soci.analyzeTrajectory(aico.q,false)<< endl;
      if(useDisplay){
        gl.watch();  
        soci.displayTrajectory(aico.q,1);
        computeTotalCost(soci,aico.q,true);
        gl.watch();
      }  
    }
    else if(useDisplay)gl.watch();
    
    listDelete(soci.s->vars);
  }
}
