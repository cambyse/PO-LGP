#include "al_gui.h"

#include <MT/opengl.h>
#include <MT/ors.h>
#include <relational/robotManipulationSimulator.h>

class sGui {
  public:
    OpenGL gl;
    ors::Graph ors;

    sGui(const char* orsFile);
  
};

void drawEnv(void*){  glStandardLight(NULL); }

sGui::sGui(const char* orsFile){
  ors.init(orsFile);
  gl.add(drawEnv,0);
  gl.add(ors::glDrawGraph,&ors);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.camera.upright();
  gl.update();
}

Gui::Gui(const char* orsFile) : Process("Gui Process"), 
  s(new sGui(orsFile))
{ }

void Gui::open() {}

void Gui::step() {
  guiData->readAccess(this);
  if(guiData->sample) {
    relational::generateOrsFromSample(s->ors, *guiData->sample);
    s->ors.calcBodyFramesFromJoints();
  }
  guiData->deAccess(this);
  s->gl.timedupdate(0.01);
}

void Gui::close() {}

Gui::~Gui() {
  delete s;  
}
