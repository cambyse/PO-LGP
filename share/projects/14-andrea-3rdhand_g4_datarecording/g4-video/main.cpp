#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/videoEncoder.h>
#include <Perception/g4data.h>

void setup_opengl_for_g4(mlr::KinematicWorld& ors, OpenGL& gl, uint hubs){
  bindOrsToOpenGL(ors, gl);
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();

  mlr::Shape *s = new mlr::Shape(ors, NoBody);
  s->type = mlr::ST_marker;
  s->size[0] = .5;

  for(uint m=0;m<hubs;m++){
    mlr::Shape *s = new mlr::Shape(ors, NoBody);
    s->type = mlr::ST_box;
    memmove(s->size ,ARR(.10, .04, .01, 0).p, 4*sizeof(double));
    memmove(s->color,ARR(1, 0, 0).p, 3*sizeof(double));
  }
}

void display(G4Data &g4d) {
  uint numF = g4d.numFrames();
  uint numS = g4d.id().sensors().N;

  arr pos = g4d.query("pos");
  arr quat = g4d.query("quat");

  String videoname;
  mlr::getParameter(videoname, "video");
  VideoEncoder_x264_simple video(videoname, 120, 0, true);

  OpenGL gl;
  mlr::KinematicWorld ors;
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
  mlr::initCmdLine(argc, argv);

  G4Data g4d;

  // load data
  mlr::String meta, poses;
  // TODO remmeber that you changed the g4data load function.. now meta and poses files have specific names
  mlr::getParameter(meta, "meta");
  mlr::getParameter(poses, "poses");
  g4d.load("./", true);

  display(g4d);
}

