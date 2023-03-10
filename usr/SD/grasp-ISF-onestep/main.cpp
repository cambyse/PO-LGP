#include <MT/soc.h>
#include <Core/array.h>
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

void
createISPTaskVariables(soc::SocSystem_Ors& sys, GraspObject *graspobj){
  createStandardRobotTaskVariables(sys);
  
  mlr::Array<mlr::Shape*> tipsN, fingN;
  mlr::Shape *palm; // palm center wrt wrist body

  /* ------- Task Vars -------- */
  TaskVariableList TVs_all; 
  TaskVariable *TV_tipAlign;
  TaskVariable *TV_palm;
  TaskVariable *TV_palmAlignDir;
  TaskVariable *TV_palmAlignField;
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
  palm = sys.ors->getShapeByName("graspCenter");

  /* finger tips aligned with the negative gradient of the potential field */
  TV_tipAlign = new PotentialFieldAlignTaskVariable("tips z align",
      *sys.ors, tipsN, *graspobj);
  /* position of the palm center marker */
  mlr::Array<mlr::Shape*> palmL; palmL.append(palm);
  TV_palm = new PotentialValuesTaskVariable("palm pos",
      *sys.ors, palmL, *graspobj);
  TV_palmAlignField = new PotentialFieldAlignTaskVariable("palm ori",
      *sys.ors, palmL, *graspobj);
  /* use this to generate different approach directions */
  TV_palmAlignDir = new DefaultTaskVariable("appr dir",
      *sys.ors, zalignTVT, "m9", "<d(0 0 0 1)>", 0, 0, 0);
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

  TVs_all.append({ TV_zeroLevel, TV_ISFcol,  TV_opp_fng, TV_opp_tip, TV_palm });
  TVs_all.append({ TV_palmAlignDir, TV_palmAlignField, TV_tipAlign, TV_col, TV_lim, TV_q});

  sys.setTaskVariables(TVs_all);

}

#define SD_PAR_R(n)  mlr::getParameter<double>((n));

void setISPGraspGoals(soc::SocSystem_Ors& sys,uint T, GraspObject *graspobj){
  sys.setx0ToCurrent();

  /* configuration */
  static bool firstTime=true;
  static double tv_palm_prec,
                tv_palmAlign_prec,
                tv_appr_dir_prec,
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
    tv_palmAlign_prec =   SD_PAR_R("grasp_palmAlign_prec");
    tv_appr_dir_prec =    SD_PAR_R("grasp_tv_appr_dir_prec");
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
  CHECK_EQ(T,sys.nTime(),"");

  //deactivate all variables
  activateAll(sys.vars,false);

  TaskVariable *V;

  V=listFindByName(sys.vars,"tips z align");
  V->updateState();
  V->y_target = ARR(-1.,-1.,-1.);
  V->setInterpolatedTargetsEndPrecisions(T,0,tv_tipAlign_prec,0.,0.);

  V=listFindByName(sys.vars,"palm pos");
  V->updateState();
  V->y_target = ARR(-.01);
  V->setInterpolatedTargetsEndPrecisions(T,0,tv_palm_prec,0.,0.);

  V=listFindByName(sys.vars,"palm ori");
  V->updateState();
  V->y_target = ARR(-1.);
  V->setInterpolatedTargetsEndPrecisions(T,0,tv_palmAlign_prec,0.,0.);

  V=listFindByName(sys.vars,"appr dir");
  V->updateState();
  V->y_target = 0;
  V->setInterpolatedTargetsEndPrecisions(T,0,tv_appr_dir_prec,0.,0.);

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
  V->setConstTargetsConstPrecisions(T,0.,0.);
  V->y_prec_trajectory(T) = tv_zeroLevel_prec;
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

  double gp_size=mlr::Parameter<double>("gp_size");
  arr pts, grads, mins, maxs, c;
  uint i;      

  c=mlr::Parameter<arr>("center");

  /* GP for random object generation and for learning */
  GraspObject_GP *ot = new GraspObject_GP( c, gp_size);
  GraspObject_GP *oe = new GraspObject_GP( c, gp_size);

  /* generate object using sampling from GP */
  rnd.seed(mlr::Parameter<uint>("rnd_srfc_seed"));
  randomGP_on_random_points(ot->isf_gp.gp, c, gp_size, 5);
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
  mlr::open(f_gp,mlr::getParameter<mlr::String>("gp_file"));
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
  listFindByName(sys.vars,"appr dir")->active=true; 
  listFindByName(sys.vars,"palm ori")->active=true; 
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
  listFindByName(sys.vars,"palm ori")->active=true; 
  listFindByName(sys.vars,"appr dir")->active=true; 
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
  arr c=mlr::Parameter<arr>("center");
  double gp_size=mlr::Parameter<double>("gp_size");
  switch (mlr::getParameter<uint>("shape")){
    case 0: o = new GraspObject_Sphere();break;
    case 1: o = new GraspObject_InfCylinder();break;
    case 2: o = new GraspObject_Cylinder1();break;
    case 3: o = random_obj(); break;
    case 4: o = new GraspObject_GP(c,gp_size);
            std::ifstream f_gp;
            mlr::open(f_gp,mlr::getParameter<mlr::String>("gp_file"));
            ((GraspObject_GP*)o)->isf_gp.read(f_gp);
            f_gp.close();
            ((GraspObject_GP*)o)->isf_gp.gp.recompute();
            o->m.readFile("a.tri");
            break;
  }
  uint T=mlr::getParameter<uint>("reachPlanTrajectoryLength");
  double t=mlr::getParameter<double>("reachPlanTrajectoryTime");
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
  soc::SocSystem_Ors sys,sys2;
  mlr::KinematicWorld ors;
  OpenGL gl;
  GraspObject *o;
  arr c=mlr::Parameter<arr>("center");
  arr tr=mlr::Parameter<arr>("translation");
  double gp_size=mlr::Parameter<double>("gp_size");
  switch (mlr::getParameter<uint>("shape")){
    case 0: o = new GraspObject_Sphere();break;
    case 1: o = new GraspObject_InfCylinder();break;
    case 2: o = new GraspObject_Cylinder1();break;
    case 5: o = new GraspObject_Box();break;
    case 3: o = random_obj(); break;
    case 4: o = new GraspObject_GP(c,gp_size);
            std::ifstream f_gp;
            mlr::open(f_gp,mlr::getParameter<mlr::String>("gp_file"));
            ((GraspObject_GP*)o)->isf_gp.read(f_gp);
            f_gp.close();
            ((GraspObject_GP*)o)->isf_gp.translate_gp_input(tr);
            ((GraspObject_GP*)o)->isf_gp.gp.recompute();
            o->m.readFile("a.tri");
            o->m.translate(tr(0),tr(1),tr(2));
            break;
  }
  uint T=mlr::getParameter<uint>("reachPlanTrajectoryLength");
  double t=mlr::getParameter<double>("optTimeMin"); 
  double t_min=mlr::getParameter<double>("optTimeMin"); 
  double alpha=mlr::getParameter<double>("alpha");
  double BinvFactor=mlr::getParameter<double>("BinvFactor");
  double tm;
  arr q0, b,b0,B, Binv, r,R;
  arr zero14(14);zero14.setZero();

  if (!o->m.V.N)  o->buildMesh();
  ors.init(mlr::getParameter<mlr::String>("orsFile"));
  mlr::Shape *pc = new mlr::Shape(ors, ors.getBodyByName("OBJECTS"));
  pc->mesh = o->m; /* add point cloud to ors */
  pc->type = mlr::ST_pointCloud; 
  gl.add(glDrawMeshObject, o);
  gl.add(glDrawPlot,&plotModule); // eureka! we plot field
  gl.add(glStandardScene);
  gl.add(mlr::glDrawGraph, &ors);
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

  double task_eps = mlr::getParameter<double>("oneStep_tascCost_eps");

/* try 3 diferent approach vectors (ortogonal to) */
  mlr::Array<char*> ax; ax.append("<d(90 0 0 1)>"); ax.append("<d(90 1 0 0)>"); ax.append("<d(90 0 1 0)>"); 
  double task_cost, best_task_cost = 1e100;
  arr best_b0 = NULL;
  for(uint i=0; i<3; ++i){ 


    sys.setq(q0,0);

    ((DefaultTaskVariable*)listFindByName(sys.vars,"appr dir"))->jrel.setText(ax(i)) ;
    activateVars_1step(sys);
    int guenter;
    OneStepDynamicFull(b,B,guenter, sys, t/*t,t_min,tm*/,alpha,task_eps,1e-3,false, false); 
//     void OneStepDynamicFull(arr& b,arr& Binv,int& counter,
//                         soc::SocSystemAbstraction& sys,
//                         double time,double alpha,double task_eps,double eps_alpha,
//                         bool verbose, bool b_is_initialized);
    
    /* open fingers */
    b.subRef(7,13) = ARR(.5,-1.,.4,-1.2,.4,-1.2,.4);
    sys.setx(b);
    sys.gl->watch("Belief after 1st phase, with open fings");
    MLR_MSG( "Post belief1 :" << b); //MLR_MSG( "time:" << tm);

    activateVars_2step(sys);
    OneStepDynamicFull(b,B,guenter, sys, t,alpha,task_eps,1e-3,true,false); 
    MLR_MSG( "Post belief2 :" << b); //MLR_MSG( "time:" << tm);

    /* see bwdMsg */
    b0.setCarray(b.p,14);
    sys.setq(b0,0);
    sys.gl->watch("Belief after 2nd phase.");


    task_cost = sys.taskCost(NULL, T, -1, 0);
    MLR_MSG( "task_cost:" << task_cost);
    if ( task_cost <  best_task_cost ){
      best_task_cost = task_cost;
      best_b0 = b0;
    }
  }

  /* see best bwdMsg */
  sys.setq(best_b0,0);
  gl.watch("best bwdMsg in terms of task Cost");

  /* start aico with bwdMsg */
  sys2.initBasics(&ors,NULL,&gl,T,/*0.6*tm*/t,true,NULL);
  createISPTaskVariables(sys2,o);
  setISPGraspGoals(sys2,T,o);
  /* use full set of TVs */
  activateVars_2step(sys2);
  /* set start position 0; */
  sys2.setq(q0); sys2.setx0ToCurrent();

  AICO solver(sys2);
  solver.useBwdMsg=true;
  solver.bwdMsg_v = cat(best_b0,zero14);
  inverse(Binv,B);
  solver.bwdMsg_Vinv.setDiag(BinvFactor,28); //=BinvFactor*Binv;
  MLR_MSG("Vinv="<<solver.bwdMsg_Vinv);
  //solver.iterate_to_convergence(); //roll out cycle
  for(uint k=0;k<solver.max_iterations;k++){
    double d=solver.step();
    if(k && d<solver.tolerance) break;

    BinvFactor *= .75;
    solver.bwdMsg_Vinv.setDiag(BinvFactor,28);
  }
  sys2.getTaskCosts(R,r,solver.q[T],T);
  MLR_MSG( "last q:"<< solver.q[T]);

}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  int mode=mlr::getParameter<int>("mode");
  switch(mode){
  case 4:  problem4();  break;
  case 5:  problem5();  break;
  default: NIY;
  }
  return 0;
}

