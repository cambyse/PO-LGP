#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
#include "fgplot.h"

#include <unistd.h>

void lib_hardware_G4();

void init_display(ors::KinematicWorld &kw) {
  MT::String shapes = MT::getParameter<MT::String>("shapes");
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
  String b1("rh:thumb"), b2("bottle");
  uint wlen = 121;

  KeyValueGraph kvg;
  //kf.EM(kvg, b1, b2, wlen);
  kf.EM(kvg, b1, wlen);

  FGPlots fgp;
  fgp.open(kvg);
  uint F = g4d.getNumFrames();
  for(uint f = 0; f < F; f++) {
    kf.updateOrs(f);
    fgp.step(f);
  }

  // TODO this is bullshit
  //KeyFrameL kfs = kf.getKeyFrames(vit);
  //kf.saveKeyFrameScreens(kfs, 30);
  //cout << "Tot num keyframes: " << kfs.N << endl;

  //gl.watch();
}

void loadData(G4Data &g4d) {
  MT::String meta = MT::getParameter<MT::String>("meta");
  MT::String poses = MT::getParameter<MT::String>("poses");
  g4d.loadData(meta, poses, true);
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  G4Data g4d;
  loadData(g4d);
  display(g4d);

  return 0;
}

