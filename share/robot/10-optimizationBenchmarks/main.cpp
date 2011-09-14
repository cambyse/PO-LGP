#include <MT/soc.h>
#include <MT/array.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h>
#include "SD/potentialTaskVariables.h"
#include "SD/miscTaskVariables.h"
#include "SD/graspObjects.h"
#include "SD/surface_helpers.h"
#include "DZ/aico_key_frames.h"

//===========================================================================

void problem1(){
  cout <<"\n= problem 1: simple kinematic reaching with 1 TV =\n" <<endl;

  //setup the system
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,3.,false,NULL);
  
  //setup the task
  TaskVariable *pos = new TaskVariable("position" , *sys.ors, posTVT, "palmCenter", 0, ARR());
  sys.setTaskVariables(ARRAY(pos));
  pos->y_target = arr(sys.ors->getShapeByName("target")->X.pos.p,3);
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,0.);

  AICO solver;

  cout <<"\n== second test: T step planning ==\n" <<endl;
  T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.setTimeInterval(3.,T);
  sys.setToq0();
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,10*1e4);
  solver.init(sys);
  solver.iterate_to_convergence();
}

//===========================================================================

void problem2(){
  cout <<"\n= problem 2: dynamic reaching =\n" <<endl;

  //setup the system
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  //setup the task
  TaskVariable *pos = new TaskVariable("position" , *sys.ors, posTVT, "palmCenter", 0, ARR());
  sys.setTaskVariables(ARRAY(pos));
  pos->y_target = arr(sys.ors->getShapeByName("target")->X.pos.p,3);
  pos->setInterpolatedTargetsEndPrecisions(T,
                                           MT::getParameter<double>("reachPlanMidPrec"),
                                           MT::getParameter<double>("reachPlanEndPrec"),
                                           0.,
                                           MT::getParameter<double>("reachPlanEndVelPrec"));

  AICO solver;
  solver.init(sys);
  solver.iterate_to_convergence();
}


//===========================================================================

void problem3(){
  cout <<"\n= problem 3: dynamic grasping =\n" <<endl;

  //setup the problem
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  createStandardRobotTaskVariables(sys);
  setGraspGoals(sys,T,"cyl1");

  AICO solver(sys);
  solver.iterate_to_convergence();
}

//===========================================================================

void
createISPTaskVariables(soc::SocSystem_Ors& sys, GraspObject *graspobj){
  createStandardRobotTaskVariables(sys);
  
  MT::Array<ors::Shape*> tipsN, fingN;
  ors::Shape *palm; // palm center wrt wrist body

  /* ------- Task Vars -------- */
  TaskVariableList TVs_all; 
  TaskVariable *TV_tipAlign;
  TaskVariable *TV_palm;
  TaskVariable *TV_opp_tip;
  TaskVariable *TV_opp_fng;
  TaskVariable *TV_zeroLevel;
  TaskVariable *TV_ISFcol;
  TaskVariable *TV_col, *TV_q,   *TV_lim ;

  /* finger tip markers.  Need to be defined in the ors file. see
   * .../test/graspISF/schunk.ors for example */
  tipsN.append(sys.ors->getShapeByName("tipNormal1"));
  tipsN.append(sys.ors->getShapeByName("tipNormal2"));
  tipsN.append(sys.ors->getShapeByName("tipNormal3"));
  fingN.append(sys.ors->getShapeByName("fingNormal1"));
  fingN.append(sys.ors->getShapeByName("fingNormal2"));
  fingN.append(sys.ors->getShapeByName("fingNormal3"));

  /* misleading name -- this is the name of the red ball marker 10cm in
   * front of the palm surface. see schunk.ors in graspISF directory */
  palm = sys.ors->getShapeByName("palmCenter");

  /* finger tips aligned with the negative gradient of the potential field */
  TV_tipAlign = new PotentialFieldAlignTaskVariable("tips z align",
      *sys.ors, tipsN, *graspobj);
  /* position of the palm center marker */
  MT::Array<ors::Shape*> palmL; palmL.append(palm);
  TV_palm = new PotentialValuesTaskVariable("palm pos",
      *sys.ors, palmL, *graspobj);
   /* */
  /* opposing fingers: heuristic for a good grasp (sort of weak closure
   * argument) */
  TV_opp_tip = new zOpposeTaskVariable("oppose tip",*sys.ors, tipsN);
  TV_opp_fng = new zOpposeTaskVariable("oppose fng",*sys.ors, fingN);
  /* the value of the potential field should be 0 at fingertips */
  TV_zeroLevel = new PotentialValuesTaskVariable("zeroLevel",
      *sys.ors, tipsN, *graspobj);

  TV_ISFcol = new PotentialValuesTaskVariable("isf col",
      *sys.ors, tipsN, *graspobj);

  /* feasibility and smoothness costs constraints */
  TV_col  = listFindByName(sys.vars,"collision"); 
  TV_q    = listFindByName(sys.vars,"qitself"); 
  TV_lim  = listFindByName(sys.vars,"limits"); 

  TVs_all.append(ARRAY( TV_zeroLevel, TV_ISFcol,  TV_opp_fng, TV_opp_tip, TV_palm, TV_tipAlign,
        TV_col, TV_lim, TV_q));

  sys.setTaskVariables(TVs_all);

}

#define SD_PAR_R(n)  MT::getParameter<double>((n));

void setISPGraspGoals(soc::SocSystem_Ors& sys,uint T, GraspObject *graspobj){
  sys.setq0AsCurrent();

  /* configuration */
  static bool firstTime=true;
  static double tv_palm_prec,
                tv_opp_fng_prec,
                tv_opp_tip_prec,
                tv_zeroLevel_prec,
                tv_ISF_col_prec,
                tv_tipAlign_prec,
                comfPrec,
                endVelPrec,
                midPrec,
                limPrec,
                colPrec;

  if(firstTime){
    firstTime=false;
    tv_palm_prec =        SD_PAR_R("grasp_tv_palm_prec");
    tv_opp_tip_prec =     SD_PAR_R("grasp_tv_opp_tip_prec");
    tv_opp_fng_prec =     SD_PAR_R("grasp_tv_opp_fng_prec");
    tv_zeroLevel_prec =   SD_PAR_R("grasp_tv_zeroLevel_prec");
    tv_ISF_col_prec =     SD_PAR_R("grasp_tv_ISF_col_prec");
    tv_tipAlign_prec =    SD_PAR_R("grasp_tv_tipAlign_prec");
    colPrec =	            SD_PAR_R("reachPlanColPrec");
    comfPrec =          	SD_PAR_R("reachPlanHomeComfort");
    endVelPrec =          SD_PAR_R("reachPlanEndVelPrec");
    midPrec =             SD_PAR_R("reachPlanMidPrec");
    limPrec =             SD_PAR_R("reachPlanLimPrec");
  }

  //set the time horizon
  CHECK(T==sys.nTime(),"");

  //deactivate all variables
  activateAll(sys.vars,false);

  TaskVariable *V;

  V=listFindByName(sys.vars,"tips z align");
  V->updateState();
  V->y_target = ARR(-1.,-1.,-1.);
  V->setInterpolatedTargetsEndPrecisions(T,0,tv_tipAlign_prec,0.,0.);

  V=listFindByName(sys.vars,"palm pos");
  V->updateState();
  V->y_target = ARR(0);
  V->setInterpolatedTargetsEndPrecisions(T,0,tv_palm_prec,0.,0.);

  V=listFindByName(sys.vars,"oppose tip");
  V->updateState();
  V->y_target = 0;
  V->setInterpolatedTargetsEndPrecisions(T,0,tv_opp_tip_prec,0.,0.);

  V=listFindByName(sys.vars,"oppose fng");
  V->updateState();
  V->y_target = 0;
  V->setInterpolatedTargetsEndPrecisions(T,0,tv_opp_fng_prec,0.,0.);

  V=listFindByName(sys.vars,"zeroLevel");
  V->updateState();
  V->y_target = ARR(0.,0.,0.); 
  V->v_target = ARR(-.05,-.05,-.05); 
  V->setInterpolatedTargetsEndPrecisions(T,0.,tv_zeroLevel_prec,0.,0.);
  V->setConstantTargetTrajectory(T);
  /* set approaching velocity  for last steps*/
  uint t,M=T/8;
  for(t=T-M;t<T;t++){
    V->v_prec_trajectory(t) = tv_ISF_col_prec;
  } 

  //col lim and relax
  V=listFindByName(sys.vars,"collision"); V->y=0.;  V->y_target=0.;
  V->setInterpolatedTargetsConstPrecisions(T,colPrec,0.);
  V=listFindByName(sys.vars,"limits"); V->y=0.; V->y_target=0.; 
  V->setInterpolatedTargetsConstPrecisions(T,limPrec,0.);
  V=listFindByName(sys.vars,"qitself");
  V->y=0.; V->y_target=V->y;  V->v=0.;  V->v_target=V->v;
  V->setInterpolatedTargetsEndPrecisions(T,comfPrec,comfPrec,0,endVelPrec);
}

GraspObject_GP *
random_obj(){

  double gp_size=MT::Parameter<double>("gp_size");
  arr pts, grads, mins, maxs, c;
  uint i;      

  c=MT::Parameter<arr>("center");

  /* GP for random object generation and for learning */
  GraspObject_GP *ot = new GraspObject_GP( c, gp_size);
  GraspObject_GP *oe = new GraspObject_GP( c, gp_size);

  /* generate object using sampling from GP */
  rnd.seed(MT::Parameter<uint>("rnd_srfc_seed"));
  randomGP_on_random_points(ot->isf_gp.gp, c, gp_size, 20);
  ot->isf_gp.gp.recompute();
  /* too expensive: ot->buildMesh();
  ot->getEnclRect(mins,maxs);*/
  mins = c-.3; maxs = c+.3;

  /* estimate generated object by other GP */
  get_observs_gradwalk(pts,grads, ot, mins, maxs, 20);
  FOR1D(pts,i){
    oe->isf_gp.gp.appendGradientObservation(pts[i], grads[i]);
    oe->isf_gp.gp.appendObservation(pts[i], 0);
  }
  oe->isf_gp.gp.recompute();

  /* get estimation quality */
  SD_DBG("(V_common - V_false) / V_true = "<<ISF_common_volume(ot,oe));

  SD_DBG("saving estimated object to file...");
  std::ofstream f_gp;
  MT::open(f_gp,MT::getParameter<MT::String>("gp_file"));
  oe->isf_gp.write(f_gp);
  f_gp.close();
  oe->buildMesh();
  oe->m.writeTriFile("a.tri");

  return oe;
}

void
activateVars_1step(soc::SocSystem_Ors &sys){
  activateAll(sys.vars,false);
  listFindByName(sys.vars,"palm pos")->active=true; 
  /*
  listFindByName(sys.vars,"isf col")->active=true; 
  listFindByName(sys.vars,"tips z align")->active=true; 
  listFindByName(sys.vars,"collision")->active=true; 
  listFindByName(sys.vars,"qitself")->active=true; 
  listFindByName(sys.vars,"limits")->active=true; 
  listFindByName(sys.vars,"oppose tip")->active=true; 
  listFindByName(sys.vars,"oppose fng")->active=true; 
  listFindByName(sys.vars,"zeroLevel")->active=true; 
  */
}

void
activateVars_2step(soc::SocSystem_Ors &sys){
  activateAll(sys.vars,false);
  listFindByName(sys.vars,"palm pos")->active=true; 
  listFindByName(sys.vars,"tips z align")->active=true; 
  listFindByName(sys.vars,"collision")->active=true; 
  listFindByName(sys.vars,"qitself")->active=true; 
  listFindByName(sys.vars,"limits")->active=true; 
  listFindByName(sys.vars,"oppose tip")->active=true; 
  listFindByName(sys.vars,"oppose fng")->active=true; 
  listFindByName(sys.vars,"zeroLevel")->active=true; 
  /*
  listFindByName(sys.vars,"isf col")->active=true; 
  */
}

void
setzeroprec(soc::SocSystem_Ors &sys, uint T){
  uint i;

  FOR1D(sys.vars, i){
    sys.vars(i)->setInterpolatedTargetsConstPrecisions(T,0.,0.);
  }
}

//===========================================================================

void problem4(){
  cout <<"\n= grasping with impl. surf. potentials =\n" <<endl;

  //setup the problem
  soc::SocSystem_Ors sys;
  OpenGL gl;
  GraspObject *o;
  arr c=MT::Parameter<arr>("center");
  double gp_size=MT::Parameter<double>("gp_size");
  switch (MT::getParameter<uint>("shape")){
    case 0: o = new GraspObject_Sphere();break;
    case 1: o = new GraspObject_InfCylinder();break;
    case 2: o = new GraspObject_Cylinder1();break;
    case 3: o = random_obj(); break;
    case 4: o = new GraspObject_GP(c,gp_size);
            std::ifstream f_gp;
            MT::open(f_gp,MT::getParameter<MT::String>("gp_file"));
            ((GraspObject_GP*)o)->isf_gp.read(f_gp);
            f_gp.close();
            ((GraspObject_GP*)o)->isf_gp.gp.recompute();
            o->m.readFile("a.tri");
            break;
  }
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  double t=MT::getParameter<double>("reachPlanTrajectoryTime");
  sys.initBasics(NULL,NULL,&gl,T,t,true,NULL);
  if (!o->m.V.N)  o->buildMesh();
  gl.add(glDrawMeshObject, o);
  gl.add(glDrawPlot,&plotModule); // eureka! we plot field
  gl.watch("The object");
  
  createISPTaskVariables(sys,o);
  setISPGraspGoals(sys,T,o);

  AICO solver(sys);
  //solver.iterate_to_convergence();
  for(uint k=0;k<solver.max_iterations;k++){
    double d=solver.step();
    if(k && d<solver.tolerance) break;
  }
}


//===========================================================================

void problem5(){
  cout <<"\n= grasping with impl. surf. potentials and time optimization =\n" <<endl;

  //setup the problem
  soc::SocSystem_Ors sys;
  ors::Graph ors;
  OpenGL gl;
  GraspObject *o;
  arr c=MT::Parameter<arr>("center");
  double gp_size=MT::Parameter<double>("gp_size");
  switch (MT::getParameter<uint>("shape")){
    case 0: o = new GraspObject_Sphere();break;
    case 1: o = new GraspObject_InfCylinder();break;
    case 2: o = new GraspObject_Cylinder1();break;
    case 3: o = random_obj(); break;
    case 4: o = new GraspObject_GP(c,gp_size);
            std::ifstream f_gp;
            MT::open(f_gp,MT::getParameter<MT::String>("gp_file"));
            ((GraspObject_GP*)o)->isf_gp.read(f_gp);
            f_gp.close();
            ((GraspObject_GP*)o)->isf_gp.gp.recompute();
            o->m.readFile("a.tri");
            break;
  }
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  double t=MT::getParameter<double>("optTimeMin"); 
  double t_min=MT::getParameter<double>("optTimeMin"); 
  double alpha=MT::getParameter<double>("alpha");
  double BinvFactor=MT::getParameter<double>("BinvFactor");
  double tm;
  arr q0, b,b0,B, Binv, r,R;
  arr zero14(14);zero14.setZero();

  if (!o->m.V.N)  o->buildMesh();
  ors.init(MT::getParameter<MT::String>("orsFile"));
  ors::Shape *pc = new ors::Shape(ors.shapes, ors.getBodyByName("OBJECTS"));
  pc->mesh = o->m; /* add point cloud to ors */
  pc->type = ors::pointCloudST; 
  gl.add(glDrawMeshObject, o);
  gl.add(glDrawPlot,&plotModule); // eureka! we plot field
  gl.add(glStandardScene);
  gl.add(ors::glDrawGraph, &ors);
  gl.camera.setPosition(5,-10,10);
  gl.camera.focus(0,0,1);
  sys.initBasics(&ors,NULL,&gl,T,/*t,t_min*/t,true,NULL);

  createISPTaskVariables(sys,o);
  setISPGraspGoals(sys,T,o);

  /* get initial pose */
  sys.getq0(q0);

  /* optimal time is tm 
  GetOptimalDynamicTime(tm,b,B,sys,alpha,0.04); 
  */

  double task_eps = MT::getParameter<double>("oneStep_tascCost_eps");
  /* with optimal time, get posterior pose */
  sys.setq(q0,0);
  activateVars_1step(sys);
  OneStepDynamicFull(b,B,sys, t/*t,t_min,tm*/,alpha,task_eps,1,false); 
  /* open fingers */
  b.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  sys.setx(b);
  sys.gl->watch("Belief after 1st phase, with open fings");
  MT_MSG( "Post belief1 :" << b); //MT_MSG( "time:" << tm);

  activateVars_2step(sys);
  OneStepDynamicFull(b,B,sys, t,alpha,task_eps,1,true); 
  MT_MSG( "Post belief2 :" << b); //MT_MSG( "time:" << tm);

  /* see bwdMsg */
  b0.setCarray(b.p,14);
  sys.setq(b0,0);
  gl.watch("Enjoy the backward message");


  /* start aico with bwdMsg */
  soc::SocSystem_Ors sys2;
  sys2.initBasics(&ors,NULL,&gl,T,/*0.6*tm*/t,true,NULL);
  createISPTaskVariables(sys2,o);
  setISPGraspGoals(sys2,T,o);
  /* use full set of TVs */
  activateVars_2step(sys2);
  /* set start position 0; */
  sys2.setq(q0); sys2.setq0AsCurrent();

  AICO solver(sys2);
  solver.useBwdMsg=true;
  solver.bwdMsg_v = cat(b0,zero14);
  inverse(Binv,B);
  solver.bwdMsg_Vinv.setDiag(BinvFactor,28); //=BinvFactor*Binv;
  MT_MSG("Vinv="<<solver.bwdMsg_Vinv);
  //solver.iterate_to_convergence(); //roll out cycle
  for(uint k=0;k<solver.max_iterations;k++){
    double d=solver.step();
    if(k && d<solver.tolerance) break;

    BinvFactor *= .75;
    solver.bwdMsg_Vinv.setDiag(BinvFactor,28);
  }
  sys2.getCosts(R,r,solver.q[T],T);
  MT_MSG( "last q:"<< solver.q[T]);

}

//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  case 2:  problem2();  break;
  case 3:  problem3();  break;
  case 4:  problem4();  break;
  case 5:  problem5();  break;
  default: NIY;
  }
  return 0;
}

