#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/videoEncoder.h>

void lib_hardware_G4();

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

void display(const arr& X){
  VideoEncoder_libav_simple vid;
  OpenGL gl;
  ors::KinematicWorld ors;
  setup_opengl_for_g4(ors, gl, X.d1);

  CHECK(X.nd==3 && X.d2==7,"wrong sized g4 data set");

  for(uint t=0;t<X.d0;t++){
    for(uint b=0; b+1<ors.shapes.N && b<X.d1; b++){
      ors.shapes(b+1)->X.pos.set(X(t,b,0), X(t,b,1), X(t,b,2));
      ors.shapes(b+1)->X.rot.set(X(t,b,3), X(t,b,4), X(t,b,5), X(t,b,6));
    }
    gl.text.clear() <<"frame " <<t;
    gl.update(NULL, true);
    flip_image(gl.captureImage);
    vid.addFrame(gl.captureImage);
  }
  vid.close();
}

void loadData(arr &X){
  MT::String filename = MT::getParameter<MT::String>("filename");
  ifstream fil;
  MT::open(fil,filename);
  arr x;
  cout <<"loading ... " <<flush;
  uint t=0;
  for(;;t++){
    fil >>x;
    if(!x.N || !fil.good()) break;
    X.append(x);
  }
  X.reshape(t,X.N/t/7,7);
  cout <<X.d0 <<" frames " <<X.d1 <<" sensors loaded" <<endl;
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  arr X;
  loadData(X);
  display(X);

  return 0;
}

