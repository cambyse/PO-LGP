#include <Gui/opengl.h>
#include <Kin/kin.h>


void draw1(void*){
  glStandardLight(NULL);
  glColor(1,0,0);
  glFrontFace(GL_CW);
  glutSolidTeapot(1.);
  glDrawAxes(1.);
  glFrontFace(GL_CCW);
}


int main(int argc, const char* argv[]){

  mlr::KinematicWorld world("world.ors");

  OpenGL gl("bla",800,600);
//  gl.camera.setKinect();
//  gl.camera.X = s->X;
  gl.addDrawer(&world);
  gl.watch();

  orsDrawMarkers=orsDrawJoints=false;
  arr q;
  world.getJointState(q);
  mlr::Shape *s = world.getShapeByName("kinect");
  gl.camera.setKinect();
  for(uint i=0;i<10;i++){
    q = randn(q.N);
    world.setJointState(q);

    gl.camera.setKinect();
    gl.camera.X = s->X;

    gl.renderInBack(true, true);
    write_ppm(gl.captureImage, "z.ppm", true);
    write_ppm(gl.captureDepth, "z.d.ppm", true);
    gl.watch();

  }
//  gl.renderInBack(false, true);

//  write_ppm(gl.captureDepth, "z.ppm", true);

//  gl.watch(); //if this is commented, never ever glut/gtk is initalized

  return 0;
}
