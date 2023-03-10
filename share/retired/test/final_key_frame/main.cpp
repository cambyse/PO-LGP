#include <MT/soc.h>
#include <Core/util.h>
#include <MT/specialTaskVariables.h>
#include <Gui/opengl.h>
#include <MT/aico.h>
#include <DZ/aico_key_frames.h>

//===========================================================================

void problem1(){
  cout <<"\n= one step kinematic estimation test =\n" <<endl;

  //setup the system
  OrsSystem sys;
  OpenGL gl;
  uint T=mlr::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,3.,false,NULL);
  
  //setup the task
  TaskVariable *pos = new DefaultTaskVariable("position" , *sys.ors, posTVT, "graspCenter", 0, arr());
  sys.setTaskVariables({pos});
  pos->y_target = arr(sys.ors->getShapeByName("target")->X.pos.p,3);
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,0.);

  arr xt;
  sys.getx0(xt);
  arr b,Binv;
 
  OneStepKinematic(b,Binv,sys,1e-3,1e-4);

  sys.displayState(&b);
  gl.watch();
  
}



//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  int mode=mlr::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  default: NIY;
  }
  return 0;
}
