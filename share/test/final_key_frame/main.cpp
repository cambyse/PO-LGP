#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h>
#include <DZ/aico_key_frames.h>

//===========================================================================

void problem1(){
  cout <<"\n= one step kinematic estimation test =\n" <<endl;

  //setup the system
  OrsSystem sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,3.,false,NULL);
  
  //setup the task
  TaskVariable *pos = new DefaultTaskVariable("position" , *sys.ors, posTVT, "graspCenter", 0, ARR());
  sys.setTaskVariables(ARRAY(pos));
  pos->y_target = arr(sys.ors->getShapeByName("target")->X.pos.p,3);
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,0.);

  arr xt;
  sys.get_x0(xt);
  arr b,Binv;
 
  OneStepKinematic(b,Binv,sys,1e-3,1e-4);

  sys.displayState(&b);
  gl.watch();
  
}



//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  default: NIY;
  }
  return 0;
}
