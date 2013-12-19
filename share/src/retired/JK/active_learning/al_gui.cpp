#include "al_gui.h"

#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <relational/robotManipulationSimulator.h>

class sGui {
  public:
    OpenGL gl;
    ors::KinematicWorld ors;

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
  MT::String problem = biros().getParameter<MT::String>("problem", MT::String("tray"), this);
  guiData->readAccess(this);
  if(guiData->sample) {
    //if (problem == "tray")
      //relational::generateOrsFromTraySample(s->ors, *guiData->sample);
    //else
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
