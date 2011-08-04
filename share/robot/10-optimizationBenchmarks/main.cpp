#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h>
#include "SD/potentialTaskVariables.h"
#include "SD/miscTaskVariables.h"
#include "SD/graspObjects.h"

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

  AICO_clean solver;

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

  AICO_clean solver;
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

  AICO_clean solver(sys);
  solver.iterate_to_convergence();
}

//===========================================================================

void
createISPTaskVariables(soc::SocSystem_Ors& sys, GraspObject *graspobj){
  createStandardRobotTaskVariables(sys);
  
  MT::Array<ors::Shape*> tipsN;
  ors::Shape *palm; // palm center wrt wrist body

  /* ------- Task Vars -------- */
  TaskVariableList TVs_all; 
  TaskVariable *TV_tipAlign;
  TaskVariable *TV_palm;
  TaskVariable *TV_oppose;
  TaskVariable *TV_zeroLevel;
  TaskVariable *TV_col, *TV_q,   *TV_lim ;

  /* finger tip markers.  Need to be defined in the ors file. see
   * .../test/graspISF/schunk.ors for example */
  tipsN.append(sys.ors->getShapeByName("tipNormal1"));
  tipsN.append(sys.ors->getShapeByName("tipNormal2"));
  tipsN.append(sys.ors->getShapeByName("tipNormal3"));

  /* misleading name -- this is the name of the red ball marker 10cm in
   * front of the palm surface. see schunk.ors in graspISF directory */
  palm = sys.ors->getShapeByName("palmCenter");

  /* finger tips aligned with the negative gradient of the potential field */
  TV_tipAlign = new PotentialFieldAlignTaskVariable("tips z align",
      *sys.ors, tipsN, *graspobj);
  /* position of the palm center marker */
  TV_palm = new TaskVariable();
  /* TODO: need this? or make zeroLeel? */
  TV_palm->set("palm pos",*sys.ors, posTVT,
	       palm->body->index,palm->rel, -1, ors::Transformation(),ARR());
   /* */
  /* opposing fingers: heuristic for a good grasp (sort of weak closure
   * argument) */
  TV_oppose = new zOpposeTaskVariable("oppose",*sys.ors, tipsN);
  /* the value of the potential field should be 0 at fingertips */
  TV_zeroLevel = new PotentialValuesTaskVariable("zeroLevel",
      *sys.ors, tipsN, *graspobj);

  /* feasibility and smoothness costs constraints */
  TV_col  = listGetByName(sys.vars,"collision"); 
  TV_q    = listGetByName(sys.vars,"qitself"); 
  TV_lim  = listGetByName(sys.vars,"limits"); 

  TVs_all.append(ARRAY( TV_zeroLevel, TV_oppose, TV_palm, TV_tipAlign,
        TV_col, TV_lim, TV_q));

  sys.setTaskVariables(TVs_all);

}

#define SD_PAR_R(n)  MT::getParameter<double>((n));

void setISPGraspGoals(soc::SocSystem_Ors& sys,uint T, GraspObject *graspobj){
  sys.setq0AsCurrent();

  /* configuration */
  static bool firstTime=true;
  static double tv_palm_prec_m,
                tv_oppose_prec,
                tv_zeroLevel_prec_m,
                tv_tipAlign_prec_m,
                comfPrec,
                endVelPrec,
                midPrec,
                limPrec,
                colPrec;

  if(firstTime){
    firstTime=false;
    tv_palm_prec_m =      SD_PAR_R("grasp_tv_palm_prec_m");
    tv_oppose_prec =      SD_PAR_R("grasp_tv_oppose_prec");
    tv_zeroLevel_prec_m = SD_PAR_R("grasp_tv_zeroLevel_prec_m");
    tv_tipAlign_prec_m =  SD_PAR_R("grasp_tv_tipAlign_prec_m");
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

  V=listGetByName(sys.vars,"tips z align");
  V->setGains(.01,.0);
  V->updateState();
  V->y_target = -1.;
  V->setInterpolatedTargetsEndPrecisions(T,midPrec,tv_tipAlign_prec_m,0.,0.);

  /* */
  V=listGetByName(sys.vars,"palm pos");
  V->setGains(.1,.0);
  V->updateState();
  // TODO: set target here
  V->y_target = graspobj->center();
  // TODO: set interpolate
  V->setInterpolatedTargetsEndPrecisions(T,midPrec,tv_palm_prec_m,0.,0.);
  /* */

  V=listGetByName(sys.vars,"oppose");
  V->setGains(.1,.0);
  V->updateState();
  V->y_target = 0;
  V->setInterpolatedTargetsEndPrecisions(T,midPrec,tv_oppose_prec,0.,0.);

  V=listGetByName(sys.vars,"zeroLevel");
  V->setGains(.1,.0);
  V->updateState();
  V->y_target = ARR(0,0,0); 
  V->setInterpolatedTargetsEndPrecisions(T,midPrec,tv_zeroLevel_prec_m,0.,0.);

  /*
  */
    
  //col lim and relax
  V=listGetByName(sys.vars,"collision"); V->y=0.;  V->y_target=0.;
  V->setInterpolatedTargetsConstPrecisions(T,colPrec,0.);
  V=listGetByName(sys.vars,"limits"); V->y=0.; V->y_target=0.; 
  V->setInterpolatedTargetsConstPrecisions(T,limPrec,0.);
  V=listGetByName(sys.vars,"qitself");
  V->y=0.; V->y_target=V->y;  V->v=0.;  V->v_target=V->v;
  V->setInterpolatedTargetsEndPrecisions(T,comfPrec,0.,midPrec,endVelPrec);
}

//===========================================================================

void problem4(){
  cout <<"\n= grasping with impl. surf. potentials =\n" <<endl;

  //setup the problem
  soc::SocSystem_Ors sys;
  OpenGL gl;
  GraspObject *o = new GraspObject_Sphere();
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  o->buildMesh();
  gl.add(glDrawMeshObject, o);
  
  createISPTaskVariables(sys,o);
  setISPGraspGoals(sys,T,o);

  AICO_clean solver(sys);
  solver.iterate_to_convergence();
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
  default: NIY;
  }
  return 0;
}

