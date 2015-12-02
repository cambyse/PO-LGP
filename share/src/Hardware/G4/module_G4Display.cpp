#include "module_G4Display.h"

#include <Gui/opengl.h>
// #include <Perception/g4data.h>

REGISTER_MODULE(G4Display)

struct sG4Display {
  uint sensors;
  OpenGL gl;
  ors::KinematicWorld ors;
  MocapID *mid;

  sG4Display(): mid(nullptr) {}
};

G4Display::G4Display():Module("G4Display"){
  s = new sG4Display;
}

ors::KinematicWorld &G4Display::kw() {
  return s->ors;
}

MocapID *G4Display::mid() {
  return s->mid;
}

void G4Display::open(){
  if(s->mid)
    s->sensors = s->mid->sensors().N;
  else {
    uint nhubs = mlr::getParameter<uint>("g4_numHubs");
    s->sensors = 3 * nhubs;
  }

  bindOrsToOpenGL(s->ors, s->gl);
  s->gl.camera.setPosition(7., .5, 3.);
  s->gl.camera.focus(0, .5, .5);
  s->gl.camera.upright();

  if(s->mid) {
    //add shapes to the sensors that don't have a shape
    for(const String &sen_name: s->mid->sensors()) {
      if(!s->ors.getBodyByName(sen_name)) {
        ors::Body *b = new ors::Body(s->ors);
        b->name = sen_name;
        ors::Shape *sh = new ors::Shape(s->ors, *b);
        sh->type = ors::boxST;
        memmove(sh->size ,ARR(.10, .04, .01, 0).p, 4*sizeof(double));
        memmove(sh->color,ARR(1, 0, 0).p, 3*sizeof(double));
      }
    }
  }
  else {
    // older, didn't consider that there might be a world
    // add shapes for the sensors
    for(uint sen=0; sen<s->sensors; sen++){
      ors::Body *b = new ors::Body(s->ors);
      ors::Shape *sh = new ors::Shape(s->ors, *b);

      sh->type = ors::boxST;
      memmove(sh->size, ARR(.10, .04, .01, 0).p, 4*sizeof(double));
      memmove(sh->color, ARR(1, 0, 0).p, 3*sizeof(double));
    }
  }

  //add a marker
  ors::Body *b = new ors::Body(s->ors);
  b->name = STRING("world");
  ors::Shape *sh = new ors::Shape(s->ors, *b);
  sh->type = ors::markerST;
  sh->size[0] = .5;
}

void G4Display::close(){}

void G4Display::step(){
  uint t = poses.readAccess();
  floatA p = poses();
  poses.deAccess();

  if(!t) return; //no revision yet -> nothing to display

  if(s->mid) {
    floatA pose;
    for(const String &sen_name: s->mid->sensors()) {
      ors::Body *b = s->ors.getBodyByName(sen_name);
      pose = s->mid->query(p, sen_name);
      b->X.pos.set(pose(0), pose(1), pose(2));
      b->X.rot.set(pose(3), pose(4), pose(5), pose(6));
    }
    s->ors.calc_fwdPropagateShapeFrames();
  }
  else {
    CHECK_EQ(p.d0, s->sensors, "poses dim is wrong");
    CHECK_EQ(s->ors.shapes.N, 1+s->sensors, "ors.shapes dim is wrong")
    for(uint sen=0; sen+1<s->ors.shapes.N && sen<s->sensors; sen++){
      s->ors.shapes(sen)->X.pos.set(p(sen,0), p(sen,1), p(sen,2));
      s->ors.shapes(sen)->X.rot.set(p(sen,3), p(sen,4), p(sen,5), p(sen,6));
    }
  }
  s->gl.text.clear() <<"frame " <<t;
  s->gl.update();

}
