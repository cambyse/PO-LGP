#include "earlyVisionModule.h"


int main(int argc,char** argv) {
  mlr::initCmdLine(argc,argv);

  RobotActionInterface R;

  R.open();
  // perception
  PerceptionModule perc;
  perc.input=&R.getProcessGroup()->evis.output;
  R.getProcessGroup()->gui.perceptionOutputVar=&perc.output;
  R.getProcessGroup()->evis;

//  if(R.getProcessGroup()->openBumble){
//    perc.threadLoop();
//    MLR_MSG("Bumble successfully started.");
//  }else{
//    MLR_MSG("Could not open bumble.");
//  }

R.wait();

}
