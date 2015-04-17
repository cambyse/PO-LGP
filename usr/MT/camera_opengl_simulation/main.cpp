#include<MT/opengl.h>

byteA texImg;
static GLuint texName;

void initChecker(){
  read_ppm(texImg,"box.ppm");
  texName=glImageTexture(texImg);
}
void drawChecker(void*){
  glDrawTexQuad(texName,
                -.2, -.2, 1.0,
                -.2, -.2, 1.4,
                0.2, -.2, 1.4,
                0.2, -.2, 1.0);
}

void TEST(StereoCapture){
  ors::KinematicWorld ors;
  ors.init("capture_scene.ors");

  OpenGL gl;
  initChecker();
  gl.add(glStandardScene,NULL);
  gl.add(drawChecker,NULL);
  gl.add(ors::glDrawGraph,&ors);
  //gl.watch();

  ors::Camera leftEye,rightEye;
  leftEye .X->setText("<t(0 -1 1) d(90 1 0 0)>");
  rightEye.X->setText("<t(.1 -1 1) d(90 1 0 0)>");

  byteA imgL,imgR;
  gl.captureStereo(imgL,imgR,200,200,&leftEye,.1);
  //write_ppm(img,"z.ppm");
  //imgL.append(imgR);
  
  flip_image(imgL);
  flip_image(imgR);

  OpenGL imgview;
  imgview.watchImage(imgL,true,1.);
  imgview.watchImage(imgR,true,1.);
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  testStereoCapture();

  return 0;
}



