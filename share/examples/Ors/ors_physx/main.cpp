#include <Ors/ors.h>
#include <Ors/ors_physx.h>
#include <Gui/opengl.h>

void createScene(mlr::KinematicWorld& ors, OpenGL& gl) {
  ors.clear();
  
  for(uint k=0; k<3; k++) {
    mlr::Body *b = new mlr::Body(ors);
    b->X.setRandom();
    b->X.pos.z += 1.;
    b->name <<"rndSphere_" <<k;
    mlr::Shape *s = new mlr::Shape(ors, *b);
    s->type=mlr::ST_box;
    s->size[0]=.1; s->size[1]=.1; s->size[2]=.1; s->size[3]=.1;
  }
  for(uint k=0; k<3; k++) {
    mlr::Body *b = new mlr::Body(ors);
    b->X.setRandom();
    b->X.pos.z += 1.;
    b->name <<"thing_" <<k;
    mlr::Shape *s = new mlr::Shape(ors, *b);
    s->type=mlr::ST_sphere;
    s->size[0]=.1; s->size[1]=.1; s->size[2]=.1; s->size[3]=.1;
    //s->mesh.readFile("pin1.off");
  }
  for(uint k=0; k<10; k++) {
    mlr::Body *b = new mlr::Body(ors);
    b->X.pos.setRandom();
    b->X.pos.z += .5;
    b->name <<"thing_" <<k;
    mlr::Shape *s = new mlr::Shape(ors, *b);
    s->type=mlr::ST_mesh;
    s->mesh.readFile("pin1.off");
  }
  ors.calc_fwdPropagateFrames();
  cout <<ors <<endl;
  
  gl.add(glStandardScene,NULL);
  gl.add(mlr::glDrawGraph,&ors);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.update();
}

void TEST(OrsPhysx) {
  mlr::KinematicWorld K;
  OpenGL glPh;

  createScene(K, K.gl());
  
  glPh.add(glStandardScene, NULL);
  glPh.add(glPhysXInterface, &K.physx());
  glPh.setClearColors(1.,1.,1.,1.);
  glPh.camera.setPosition(10.,-15.,8.);
  glPh.camera.focus(0,0,1.);
  glPh.watch();
  
  for(uint t=0; t<500; t++) {
    cout <<"\r t=" <<t <<std::flush;
    K.physx().step();
    glPh.update();
    K.watch(false);
  }
}

int MAIN(int argc, char** argv) {
  testOrsPhysx();

  return 0;
}
