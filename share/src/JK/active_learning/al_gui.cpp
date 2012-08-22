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

sGui::sGui(const char* orsFile) {
  init(ors, gl, orsFile);  
}

Gui::Gui(const char* orsFile) : Process("Gui Process"), 
  s(new sGui(orsFile))
{ }

void Gui::open() {}

void Gui::step() {
  MT::String problem = birosInfo().getParameter<MT::String>("problem", this, MT::String("tray"));
  guiData->readAccess(this);
  if(guiData->sample) {
    if (problem == "tray")
      relational::generateOrsFromTraySample(s->ors, *guiData->sample);
    else
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
