#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/video.h>
#include <Perception/g4data.h>

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

  // setup sensor shapes
  /*
  Shape *sh = ors.getShapeByName("sh:sensor");
  for(auto b: ors.bodies) {
    if(b->Shapes.N == 0) {
      b->

    }
  }
  */

  uint T = g4d.getNumTimesteps();
  MT::String bname;
  for(uint t = 0; t < T; t++) {
    for(auto b: ors.bodies) {
      arr x = g4d.query(t, b->name);
      x.reshape(x.N);
      if(norm(x) != 0) {
        b->X.pos.set(x(0), x(1), x(2));
        b->X.rot.set(x(3), x(4), x(5), x(6));
      }
    }
    ors.calcShapeFramesFromBodies();
    gl.text.clear() <<"frame " <<t <<"/" <<T;
    gl.update(NULL, true);
    //flip_image(gl.captureImage);
    //vid.addFrame(gl.captureImage);
  }
  //vid.close();
}

void loadData(G4Data &g4d){
  MT::String meta = MT::getParameter<MT::String>("meta");
  MT::String poses = MT::getParameter<MT::String>("poses");
  g4d.loadData(meta, poses);
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  G4Data g4d;
  loadData(g4d);
  display(g4d);

  return 0;
}

