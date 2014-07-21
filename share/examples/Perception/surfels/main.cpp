#include <Perception/surfels.h>
#include <Gui/opengl.h>
#include <Perception/kinect2pointCloud.h>

void TEST(Surfels) {
  Surfels S;
  S.setRandom(20);

  OpenGL gl("title",640,480);
  gl.clearB=gl.clearR=gl.clearG=gl.clearA=0;
  initKinectCam();
  gl.camera=kinectCam;

  gl.add(glDrawSurfels, &S);
  gl.watch();

  S.renderIndex=true;
  gl.renderInBack(); //20,20);
  flip_image(gl.captureImage);
  const byteA& img = gl.captureImage;
  uint32A surfelIdx(img.d0,img.d1);
  byteA mask(img.d0,img.d1);
  for(uint i=0;i<surfelIdx.N;i++){
    surfelIdx.elem(i) = img.elem(3*i+0)<<16 | img.elem(3*i+1)<<8 | img.elem(3*i+2);
    mask.elem(i) = (surfelIdx.elem(i)==0?0:255);
  }

  OpenGL gl2("title",640,480);
  gl2.watchImage(mask, true, 1);
}

int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  testSurfels();

  return 0;
}
