#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

float angle;

void draw(void*){
  glStandardLight(NULL);
  glColor(1, 0, 0);
  glRotatef(angle, 0, 1, 0);
  glutSolidTeapot(1.);
}

void testVideo(){
  OpenGL gl;
  gl.add(draw,0);
  //gl.watch();

  VideoEncoder_libav_simple video;
  for(angle=0.;angle<180.;angle+=180./150.){
    gl.update(NULL, true);
    flip_image(gl.captureImage);
    video.addFrame(gl.captureImage);
  }
  video.close();
}

int main(int argc,char **argv){
  testVideo();

  return 0;
}

