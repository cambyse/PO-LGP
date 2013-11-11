#define MT_IMPLEMENTATION
//#define NIKOLAY

#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/soc.h>
int useDisplay = -1;
int nShape = -1;
//#include "NikolayGraspRoutines.h"

using namespace soc;



void drawEnv(void*){
  glStandardLight(NULL);
  //glDrawFloor(4.,1,1,1);
}
void init(ors::Graph& ors,OpenGL& gl,const char *filename){
  ors.init(filename);
  
  gl.add(drawEnv,0);
  gl.add(ors::glDrawGraph,&ors);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(.0,0.,10.);
  gl.camera.focus(.0,0,0);
}


void testPlan(){
  ors::Graph ors;
  SwiftInterface swift;
  OpenGL gl;
  nShape = MT::getParameter<int>("nShape");
  if(!nShape)
    init(ors,gl, MT::getParameter<String>("sFile1"));
  else
    init(ors,gl,MT::getParameter<String>("sFile2"));
  swift.init(ors,.05);
 //BayesianKinematicMotionPlanning(soci,q,30,.7,.001,1);
  //PlanGraspM(ors,swift,gl);
  //return;
  
  /*  ors.getName("tip1")->shapes(0)->contactOrientation.set(0,1,0);
  ors.getName("tip2")->shapes(0)->contactOrientation.set(0,1,0);
  ors.getName("tip3")->shapes(0)->contactOrientation.set(0,1,0);*/
  //ors.getName("fing1")->shapes(0)->contactOrientation.set(0,1,0);
  //ors.getName("fing2")->shapes(0)->contactOrientation.set(0,1,0);
  //ors.getName("fing3")->shapes(0)->contactOrientation.set(0,1,0);
  
  //createKinematic(soci,&ors,&swift,&gl,200);
  soc::SocSystem_Ors soci;
  soci.initBasics(&ors,&swift,&gl,200,1.,false,NULL);
  
  soc::AICO aico;
  int display=MT::getParameter<int>("reachPlanDisplay");
  double eps=MT::getParameter<double>("reachPlanEps");
  double rate=MT::getParameter<double>("reachPlanRate");
  uint K=MT::getParameter<uint>("reachPlanK");

  aico.init(soci,rate,eps,0.,display,0,NULL);
  
  for(uint k=0;k<K;k++){
    //col->setPrecisionTrajectoryConstant(T,k*colPrec);
    if(!soci.dynamic) aico.stepKinematic();
    else{
      //if(k>5){ eps=0; rate=.5; }
      if(k<K/2) aico.stepDynamic();
      else      NIY; //iLQG_general(soci,aico.q,10,rate,display);
    }
  }
  ofstream fil("z.plan");
  aico.v.writeTagged(fil,"v");
  aico.Vinv.writeTagged(fil,"Vinv");
  soci.displayTrajectory(aico.q,NULL,1,"");
  gl.watch();
}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  
  useDisplay=MT::getParameter<int>("reachPlanDisplay");
  MT::IOraw = true;
  testPlan();
}
