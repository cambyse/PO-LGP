#define MT_IMPLEMENTATION

#include <MT/ors.h>

double blooby(double x,double y,double z,void*){
  return x*x*x*x - 5*x*x+ y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8;
}

double sphere(double x,double y,double z,void*){
  return (x*x +y*y+z*z)-1.;
}

double torus(double x,double y,double z,void*){
  double r=sqrt(x*x + y*y);
  return z*z + (1.-r)*(1.-r) - .1;
}

//double sigmoid(double x){ return .5*(1.+x/(1.+::fabs(x))); }
double sigmoid(double x){ return 1./(1.+exp(-x)); }

double box(double x,double lo,double hi,double steep=10.){
  //outside=0, inside=2, border=1
  double xa = x-lo; xa*=steep;
  double xb = hi-x; xb*=steep;
  return 2.*(1.-sigmoid(xa)*sigmoid(xb));
}

double cylinder(double x,double y,double z,void*){
  return x*x + y*y + box(z,-1.,1.) - 1.;
}

int main (int argc, char **argv){
  ors::Mesh m;
  m.setImplicitSurface(blooby,NULL,-10.,10.,100);
  //m.setImplicitSurface(sphere,NULL,-2.,2.,100);
  //m.setImplicitSurface(torus,NULL,-10.,10.,100);
  //m.setImplicitSurface(cylinder,NULL,-5.,5.,100);
  
  OpenGL gl;
  gl.add(glStandardScene,NULL);
  gl.add(ors::glDrawMesh,&m);
  gl.watch();

  return 0 ;
}
