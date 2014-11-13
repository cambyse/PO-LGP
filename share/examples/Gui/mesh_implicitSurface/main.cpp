#include <Ors/ors.h>
#include <Gui/mesh.h>
#include <Gui/opengl.h>

//===========================================================================

ScalarFunction blobby = [](arr&,arr&, const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return x*x*x*x - 5*x*x+ y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8;
  };

ScalarFunction sphere=[](arr&,arr&, const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return (x*x +y*y+z*z)-1.;
  };

ScalarFunction torus = [](arr&,arr&, const arr& X){
    double x=X(0), y=X(1), z=X(2);
    double r=sqrt(x*x + y*y);
    return z*z + (1.-r)*(1.-r) - .1;
  };

//double sigmoid(double x){ return .5*(1.+x/(1.+::fabs(x))); }
double sigmoid(double x){ return 1./(1.+exp(-x)); }

double box(double x,double lo,double hi,double steep=10.){
  //outside=0, inside=2, border=1
  double xa = x-lo; xa*=steep;
  double xb = hi-x; xb*=steep;
  return 2.*(1.-sigmoid(xa)*sigmoid(xb));
}

ScalarFunction cylinder = [](arr&,arr&, const arr& X){
    double x=X(0), y=X(1), z=X(2);
    return x*x + y*y + box(z,-1.,1.) - 1.;
  };

void TEST(SimpleImplicitSurfaces) {
  ors::Mesh m;
  OpenGL gl;
  gl.add(glStandardScene,NULL);
  gl.add(ors::glDrawMesh,&m);

  MT::Array<ScalarFunction*> fcts = {&blobby, &sphere, &torus, &cylinder};

  for(ScalarFunction* f: fcts){
    m.setImplicitSurface(*f,-10.,10.,100);
    gl.watch();
  }
}

//===========================================================================

void TEST(DistanceFunctions) {
  ors::Transformation t;
  t.setRandom();
  ors::Mesh m;
  OpenGL gl;
  gl.add(glStandardScene,NULL);
  gl.add(ors::glDrawMesh,&m);

  MT::Array<ScalarFunction*> fcts = {
    new DistanceFunction_Sphere(t, 1.),
    new DistanceFunction_Box(t, 1., 2., 3.),
    new DistanceFunction_Cylinder(t, 1., 2.)
  };

  for(ScalarFunction* f: fcts){
    //-- check hessian and gradient
    for(uint i=0;i<1000;i++){
      arr x(3);
      rndUniform(x, -5., 5.);
      bool suc=true;
      suc &= checkGradient(*f, x, 1e-6);
      suc &= checkHessian(*f, x, 1e-6);
      if(!suc){
        arr g,H;
        f->fs(g,H,x); //set breakpoint here;
        HALT("x=" <<x);
      }
    }

    //-- display
    m.setImplicitSurface(*f,-10.,10.,100);
    gl.watch();
  }
}

//===========================================================================

int MAIN(int argc, char **argv){
  testSimpleImplicitSurfaces();
  testDistanceFunctions();

  return 0 ;
}
