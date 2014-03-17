#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
#include "fgplot.h"
#include <unistd.h>

void lib_hardware_G4();

void init_display(ors::KinematicWorld &kw) {
  MT::String shapes;
  MT::getParameter(shapes, "shapes");
  kw.init(shapes);
  //init(G, gl, shapes);
  kw.gl().camera.setPosition(7., .5, 3.);
  kw.gl().camera.focus(0, .5, .5);
  kw.gl().camera.upright();
}

#include <Algo/spline.h>
void display(G4Data &g4d) {
  ors::KinematicWorld kw;

  init_display(kw);

  KeyFramer kf(kw, g4d);
  String b1("rh:thumb"), b2("sbox");
  StringA bb;
  bb.append(STRING("rh:thumb"));
  //bb.append(STRING("rh:index"));
  //bb.append(STRING("rh:middle"));
  bb.append(STRING("lh:thumb"));
  //bb.append(STRING("lh:index"));
  //bb.append(STRING("lh:middle"));

  KeyValueGraph kvg;
  //kf.EM_m(kvg, b1);
  //kf.EM_r(kvg, b1, b2);
  //kf.EM_c(kvg, b1, b2);
  //kf.testSmoothing(kvg, b1, .3);

  //FGPlots fgp;
  //fgp.open(kvg);
  //uint F = g4d.getNumFrames();
  //for(uint f = 0; f < F; f++) {
    //kf.updateOrs(f);
    //fgp.step(f);
  //}

  //kf.playScene(b1);
  kf.playScene(bb);
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  G4Data g4d;

  // load data
  MT::String meta, poses, data;
  MT::getParameter(data, "data");
  MT::getParameter(meta, "meta");
  MT::getParameter(poses, "poses");

  g4d.load(data, meta, poses, true);
  display(g4d);
  g4d.save(data);

  return 0;
}

