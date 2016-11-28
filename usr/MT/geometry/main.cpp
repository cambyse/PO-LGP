#include <MT/ors.h>
#include <MT/opengl.h>

void addRndShape(mlr::KinematicWorld& ors){
  mlr::Shape *s = new mlr::Shape;
  s->X.setRandom();
  s->type = (mlr::ShapeType)rnd(4);
  for(uint i=0;i<4;i++) s->size[i] = rnd.uni(.03,.3);
  s->color[0] = 1.; s->color[1]=.3; s->color[2]=.3;
  ors.shapes.append(s);
}

int main(int argc, char** argv){
  mlr::KinematicWorld ors;
  OpenGL gl;
  gl.add(glStandardScene,NULL);
  gl.add(mlr::glDrawGraph,&ors);

  for(;;){
    addRndShape(ors);
    gl.watch();
  }
  
  return 0;
};
