#include "earlyVisionModule.h"


int main(int argn,char** argv) {
  MT::initCmdLine(argn,argv);

  RobotActionInterface R;

  R.open();
  // perception
  PerceptionModule perc;
  perc.input=&R.getProcessGroup()->evis.output;
  R.getProcessGroup()->gui.perceptionOutputVar=&perc.output;
  R.getProcessGroup()->evis;

//  if(R.getProcessGroup()->openBumble){
//    perc.threadLoop();
//    MT_MSG("Bumble successfully started.");
//  }else{
//    MT_MSG("Could not open bumble.");
//  }

R.wait();

}
