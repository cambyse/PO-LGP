#include <Perception/surfels.h>
#include <Gui/opengl.h>

void TEST(Surfels) {
  Surfels S;
  S.setRandom(20);

  OpenGL gl("title",200,200);
  gl.clearB=gl.clearR=gl.clearG=gl.clearA=0;
  gl.camera.setPosition(0., 0., 0.);
  gl.camera.focus(0., 0., 1.);
  gl.camera.setZRange(.1, 10.);
  gl.camera.setHeightAngle(45.);

  gl.add(glDrawSurfels, &S);
  gl.watch();
  S.renderIndex=true;

  gl.renderInBack(20,20);
  const byteA& img = gl.captureImage;
  uint32A idx(img.d0*img.d1);
  for(uint i=0;i<idx.N;i++) idx(i) = img.elem(3*i+0)<<16 | img.elem(3*i+1)<<8 | img.elem(3*i+2);
  idx.reshape(img.d0,img.d1);
  cout <<idx <<endl;
}

int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  testSurfels();

  return 0;
}
