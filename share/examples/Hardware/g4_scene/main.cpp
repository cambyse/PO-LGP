#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/video.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
#include "feedgnuplot.h"

#include <unistd.h>

void lib_hardware_G4();

/*
void setup_opengl_for_g4(ors::Graph& ors, OpenGL& gl, uint hubs){
  bindOrsToOpenGL(ors, gl);
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();

  ors::Shape *s = new ors::Shape(ors, NULL);
  s->type = ors::markerST;
  s->size[0] = .5;

  for(uint m=0;m<hubs;m++){
    ors::Shape *s = new ors::Shape(ors, NULL);
    s->type = ors::boxST;
    memmove(s->size ,ARR(.10, .04, .01, 0).p, 4*sizeof(double));
    memmove(s->color,ARR(1, 0, 0).p, 3*sizeof(double));
  }
}
*/

void display(const G4Data &g4d) {
  //VideoEncoder_libav_simple vid;
  OpenGL gl;
  ors::Graph ors;
  //setup_opengl_for_g4(ors, gl, X.d1);

  MT::String shapes = MT::getParameter<MT::String>("shapes");
  init(ors, gl, shapes);
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();

  // add bodies to keyframer
  KeyFramer kf;
  kf.addBody(24); // 8*3
  kf.setAgent(0);
  kf.addBody(3); // sbox
  kf.addBody(3); // bottle
  kf.addBody(3); // book
  kf.addBody(3); // ball
  kf.addBody(6); // bbox

  // add states to keyframer
  uint T = g4d.getNumTimesteps();
  for(uint t = 0; t < T; t++)
    kf.addState(g4d.queryPos(t));

  kf.setLWin(30);
  kf.run(); // trains the linear classifier

  Feedgnuplot gnup;
  gnup.open();

  MT::String bname;
  for(uint t = 0; t < T; t++) {
    for(auto &b: ors.bodies) {
      arr x = g4d.query(t, b->name);
      x.reshape(x.N); // TODO there should be a nicer way.. // resetD doesn't work
      CHECK(norm(x)!=0, "Why isn't interpolation on?");

      b->X.pos.set(x(0), x(1), x(2));
      b->X.rot.set(x(3), x(4), x(5), x(6));
    }

    ors.calcShapeFramesFromBodies();
    gl.text.clear() <<"frame " <<t <<"/" <<T;
    gl.update(NULL, true);
    //flip_image(gl.captureImage);
    //vid.addFrame(gl.captureImage);

    // TODO fix index of getErr
    if(t + 5 < T) {
      std::stringstream sss;
      for(int b = 1; b < kf.getNBodies(); b++) {
        arr TEMP = kf.getErr(b);
        sss << " " << TEMP(t, 0) << " " << TEMP(t, 1) << " " << TEMP(t, 2);
      }
      gnup() << t << sss.str();
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

