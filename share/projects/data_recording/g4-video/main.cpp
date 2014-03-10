#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/videoEncoder.h>
#include <Perception/g4data.h>

void setup_opengl_for_g4(ors::KinematicWorld& ors, OpenGL& gl, uint hubs){
  bindOrsToOpenGL(ors, gl);
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();

  ors::Shape *s = new ors::Shape(ors, NoBody);
  s->type = ors::markerST;
  s->size[0] = .5;

  for(uint m=0;m<hubs;m++){
    ors::Shape *s = new ors::Shape(ors, NoBody);
    s->type = ors::boxST;
    memmove(s->size ,ARR(.10, .04, .01, 0).p, 4*sizeof(double));
    memmove(s->color,ARR(1, 0, 0).p, 3*sizeof(double));
  }
}

void display(G4Data &g4d) {
  uint numF = g4d.getNumFrames();
  uint numS = g4d.getNumSensors();

  arr pos = g4d.query("pos");
  arr quat = g4d.query("quat");

  String videoname;
  MT::getParameter(videoname, "video");
  VideoEncoder_x264_simple video(videoname, 120, 0, true);

  OpenGL gl;
  ors::KinematicWorld ors;
  setup_opengl_for_g4(ors, gl, pos.d0);

  CHECK(pos.nd==3 && pos.d2==3,"wrong sized g4 pos dataset");
  CHECK(quat.nd==3 && quat.d2==4,"wrong sized g4 quat dataset");

  for(uint f=0;f<numF;f++){
    for(uint b=0; b+1<ors.shapes.N && b<numS; b++){
      ors.shapes(b+1)->X.pos.set(pos[b][f].p);
      ors.shapes(b+1)->X.rot.set(quat[b][f].p);
    }
    gl.text.clear() <<"frame " <<f;
    gl.update(NULL, true);
    flip_image(gl.captureImage);
    video.addFrame(gl.captureImage);
  }
  video.close();
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  G4Data g4d;

  // load data
  MT::String meta, poses;
  MT::getParameter(meta, "meta");
  MT::getParameter(poses, "poses");
  g4d.load(NULL, meta, poses, true);

  display(g4d);
}

