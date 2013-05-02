

#include <MT/ors_physx.h>
#include <MT/opengl.h>

void createOrs(ors::Graph& ors, OpenGL& gl) {
  ors.clear();
  
  for(uint k=0; k<3; k++) {
    ors::Body *b = new ors::Body(ors);
    b->X.setRandom();
    b->X.pos.z += 1.;
    b->name <<"rndSphere_" <<k;
    ors::Shape *s = new ors::Shape(ors, b);
    s->type=ors::boxST;
    s->size[0]=.1; s->size[1]=.1; s->size[2]=.1; s->size[3]=.1;
  }
  for(uint k=0; k<3; k++) {
    ors::Body *b = new ors::Body(ors);
    b->X.setRandom();
    b->X.pos.z += 1.;
    b->name <<"thing_" <<k;
    ors::Shape *s = new ors::Shape(ors, b);
    s->type=ors::sphereST;
    s->size[0]=.1; s->size[1]=.1; s->size[2]=.1; s->size[3]=.1;
    //s->mesh.readFile("pin1.off");
  }
  for(uint k=0; k<10; k++) {
    ors::Body *b = new ors::Body(ors);
    b->X.pos.setRandom();
    b->X.pos.z += .5;
    b->name <<"thing_" <<k;
    ors::Shape *s = new ors::Shape(ors, b);
    s->type=ors::meshST;
    s->mesh.readFile("pin1.off");
  }
  ors.calcShapeFramesFromBodies();
  cout <<ors <<endl;
  
  gl.add(glStandardScene,NULL);
  gl.add(ors::glDrawGraph,&ors);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.update();
}


int main(int argc, char** argv) {
  ors::Graph ors;
  OpenGL glMy, glPh;
  createOrs(ors, glMy);
  
  PhysXInterface physx;
  physx.G = &ors;
  physx.create();
  
  glPh.add(glStandardScene, NULL);
  glPh.add(glPhysXInterface, &physx);
  glPh.setClearColors(1.,1.,1.,1.);
  glPh.camera.setPosition(10.,-15.,8.);
  glPh.camera.focus(0,0,1.);
  glPh.watch();
  
  for(uint t=0; t<500; t++) {
    cout <<"\r t=" <<t <<std::flush;
    physx.step();
    glPh.update();
    glMy.update();
  }
  return 0;
}
