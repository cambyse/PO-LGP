#define MT_IMPLEMENTATION

#include <signal.h>

#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/BinaryBP.h>
#include <MT/vision.h>

#include <NP/wrappers/libcolorseg.h>
#include <NP/common.h>
#include <NP/transformations.h>
#include <NP/camera/uvccamera.h>
#include <NP/camera/bumblebee2.h>

//#include <ess.hh>


#ifdef MT_BUMBLE
typedef camera::Bumblebee2 Camera;
#else
typedef camera::UVCCamera Camera;
#endif
Camera *camP;

void test(){
  Camera cam; camP=&cam;
  cam.init();
  cam.get_matrix_q(camera_calibration);
      
  //-- create opengl scene..
  ors::Graph ors;
  ors::Body *n = new ors::Body;
  ors.bodies.append(n);
  ors::Shape *s = new ors::Shape;
  n->shapes.append(s);
  s->body=n;
  s->type = BCCYLINDER;
  s->size[2]=.05; s->size[3]=.1;
  s->color[0]=1.;
  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(ors::glDrawGraph,&ors);

  
  byteA left,right;
  for(uint t=0;;t++){
    cam.capture(left,right);
    //read_ppm(left,"data/left_0001.ppm");
    //read_ppm(right,"data/right_0001.ppm");

    arr worldP;
    localizeHsv(worldP, left, right, TUPLE<float>(.0,1.,1.),TUPLE<float>(.2,.5,.5), 3);
  
    n->X.p = ors::Vector(worldP(0),worldP(1),worldP(2));
    //n->X.r = ors::Quaternion(worldP(0),worldP(1),worldP(2));
    gl.update();
  }
}

void shutdown(int){
   camP->deinit();
   exit(0);
}

int main(int argn,char** argv){
  signal(SIGINT,shutdown);

  test();
  
  return 0;
}
