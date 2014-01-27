#include "module_G4Display.h"

#include <Gui/opengl.h>
#include <Ors/ors.h>

REGISTER_MODULE(G4Display)

struct sG4Display{
  uint sensors;
  OpenGL gl;
  ors::KinematicWorld ors;
};

G4Display::G4Display():Module("G4Display"){
  s = new sG4Display;
}

void G4Display::open(){
  s->sensors = 3 * MT::getParameter<uint>("g4_numHubs");

  bindOrsToOpenGL(s->ors, s->gl);
  s->gl.camera.setPosition(7., .5, 3.);
  s->gl.camera.focus(0, .5, .5);
  s->gl.camera.upright();

  //add shapes for the sensors
  for(uint sen=0; sen<s->sensors; sen++){
    ors::Shape *sh = new ors::Shape(s->ors, NoBody);
    sh->type = ors::boxST;
    memmove(sh->size ,ARR(.10, .04, .01, 0).p, 4*sizeof(double));
    memmove(sh->color,ARR(1, 0, 0).p, 3*sizeof(double));
  }

  //add a marker
  ors::Shape *sh = new ors::Shape(s->ors, NoBody);
  sh->type = ors::markerST;
  sh->size[0] = .5;
}

void G4Display::close(){}

void G4Display::step(){
  uint t=g4data.readAccess();
  floatA poses = g4data().poses;
  g4data.deAccess();

  if(!t) return; //no revision yet -> nothing to display

  CHECK_EQ(poses.d0, s->sensors, "poses dim is wrong");
  CHECK_EQ(s->ors.shapes.N, 1+s->sensors, "ors.shapes dim is wrong")
  for(uint sen=0; sen+1<s->ors.shapes.N && sen<s->sensors; sen++){
    s->ors.shapes(sen)->X.pos.set(poses(sen,0), poses(sen,1), poses(sen,2));
    s->ors.shapes(sen)->X.rot.set(poses(sen,3), poses(sen,4), poses(sen,5), poses(sen,6));
  }
  s->gl.text.clear() <<"frame " <<t;
  s->gl.update();

}
