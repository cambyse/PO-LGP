#include "dataStructures.h"

void glDrawPrimitives(void* classP){
  ((DisplayPrimitives*)classP)->glDraw();
}

void Plane::glDraw(){
  ors::Vector p(nx,ny,nz);
  ors::Vector axis = Vector_z ^ p;
  double angle = acos(p.z / p.length());
  p *= -c/(p*p);
  glLoadIdentity();
  glTranslatef(p.x, p.y, p.z);
  glRotated(180.*angle/MT_PI, axis.x, axis.y, axis.z);
  glDisable(GL_CULL_FACE);
  glDrawRect(0,0,0,1.);
  glLoadIdentity();
}

void CloudView::glDraw(){
  conv_PclCloud_ArrCloud(pts,cols,cloud);
  glDrawPointCloud(pts,cols);
}
