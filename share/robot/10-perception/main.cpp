//#define MT_IMPLEMENTATION

#include <signal.h>

#include <NP/camera.h>
#include <MT/earlyVisionModule.h>
#include <MT/perceptionModule.h>
#include <MT/guiModule.h>


static bool STOP=false;
void shutdown(int){
  cout <<"shutting down!" <<endl;
  STOP=true;
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  signal(SIGINT,shutdown);

  ors::Graph G;
  MT::load(G,"schunk.ors", true);
  //ors::Body * o = G.getBodyByName("OBJECTS");
  //o->X.p(0) = 100;

  MyCamera cam;
  EarlyVisionModule evis;
#if 0
  evis.input=&cam.output;
#else
  CameraImages dummyImages;
  dummyImages.loadDummyImages();
  evis.input=&dummyImages;
#endif
  PerceptionModule perc;  perc.input=&evis.output;

  GuiModule gui;  gui.cameraVar=evis.input;  gui.perceptionOutputVar=&perc.output;
  gui.createOrsClones(&G);
  /*G.getJointState(gui.q_reference);
  gui.ors2->setJointState(gui.q_reference);
  gui.ors2->calcBodyFramesFromJoints();*/

  //cam .threadLoop();
  evis.threadLoop();
  perc.threadLoop();
  gui .threadLoop();
  
  for(uint i=0;!STOP && i<1000;i++){
    MT::wait(.1);
    cout <<"\r" <<i <<flush;
  }

  gui.threadClose();
  perc.threadClose();
  evis.threadClose();
  cam.threadClose();
  
  return 0;
}
