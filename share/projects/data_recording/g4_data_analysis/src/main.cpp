#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
#include "fgplot.h"
#include <unistd.h>

const char *home = getenv("HOME");

void lib_hardware_G4();

void init_display(ors::KinematicWorld &kw) {
  MT::String shapes;
  shapes << home << "/git/mlr/share/" << MT::getParameter<MT::String>("shapes");
  kw.init(shapes);
  //init(G, gl, shapes);
  kw.gl().camera.setPosition(7., .5, 3.);
  kw.gl().camera.focus(0, .5, .5);
  kw.gl().camera.upright();
}

void display(G4Data &g4d) {
  ors::KinematicWorld kw;

  init_display(kw);

  KeyFramer kf(kw, g4d);
  String b1("rh:thumb"), b2("sbox");
  uint wlen = 61;

  KeyValueGraph kvg;
  //kf.EM_m(kvg, b1, wlen);
  //kf.EM_r(kvg, b1, b2, wlen);
  kf.EM(kvg, b1, b2, wlen);

  FGPlots fgp;
  fgp.open(kvg);
  uint F = g4d.getNumFrames();
  for(uint f = 0; f < F; f++) {
    kf.updateOrs(f);
    fgp.step(f);
  }
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  G4Data g4d;

  // load data
  MT::String base, meta, poses, data;
  base << home << "/git/mlr/share/";
  meta << base << MT::getParameter<MT::String>("meta");
  poses << base << MT::getParameter<MT::String>("poses");
  data << base << MT::getParameter<MT::String>("data");

  g4d.load(data, meta, poses, true);
  display(g4d);
  g4d.save(data);

  return 0;
}

