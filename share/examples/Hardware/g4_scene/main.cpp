#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/video.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
#include "feedgnuplot.h"

#include <unistd.h>

void lib_hardware_G4();

void display(const G4Data &g4d) {
  //VideoEncoder_libav_simple vid;
  OpenGL gl;
  ors::Graph ors;

  MT::String shapes = MT::getParameter<MT::String>("shapes");
  init(ors, gl, shapes);
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();

  // add bodies to keyframer
  KeyFramer kf;
  for(int i = 0; i < 8; i++)
    kf.addBody(3);
  kf.setAgent(0);
  kf.addBody(3); // sbox
  kf.addBody(3); // bottle
  kf.addBody(3); // book
  kf.addBody(3); // ball
  kf.addBody(3); // bbox:body
  kf.addBody(3); // bbox:lid

  // add states to keyframer
  uint T = g4d.getNumTimesteps();
  for(uint t = 0; t < T; t++)
    kf.addState(g4d.queryPos(t));

  //uint wlen = 20;
  //arr corr = kf.getCorr(0, 8, wlen);
  //arr corr = kf.getCorr2(3, 8, wlen);

  uintA wlens = { 10, 20, 30, 40, 50, 60 };
  uint mwlen = wlens.max();
  MT::Array<arr> corr = kf.getCorr3(3, 8, wlens);
  
  Feedgnuplot gnup;
  gnup.open();

  MT::String bname;
  for(uint t = 0; t < T; t++) {
    for(auto &b: ors.bodies) {
      arr x = g4d.query(t, b->name);
      x.reshape(x.N); // TODO there should be a nicer way.. // resetD doesn't work
      CHECK(length(x)!=0, "Why isn't interpolation on?");

      b->X.pos.set(x(0), x(1), x(2));
      b->X.rot.set(x(3), x(4), x(5), x(6));
    }

    ors.calcShapeFramesFromBodies();
    gl.text.clear() <<"frame " <<t <<"/" <<T;
    gl.update(NULL, true);
    //flip_image(gl.captureImage);
    //vid.addFrame(gl.captureImage);

    if(t>=mwlen) {
      uint wi = t-mwlen;

      cout << "1";
      for(uint wii = 0; wii < wlens.N; wii++)
        cout << " * " << corr(wii).elem(wi);
      cout << " = " << c << endl;

      double c = 1;
      for(uint wii = 0; wii < wlens.N; wii++)
        c *= corr(wii).elem(wi);

      gnup() << t << " " << c;
    }
  }
  usleep(1000000);

  //gl.watch();
  //vid.close();
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

