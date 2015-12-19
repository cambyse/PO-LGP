#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>

#include <Optim/optimization.h>


//===========================================================================

void TEST(SCBoxFit){
  arr X(100,3);
  rndUniform(X, 2., 3.);

  arr x(11);
  rndUniform(x.subRange(0,3)(), 20., 20.);
  rndGauss(x.subRange(4,-1)());
  x.subRange(7,-1)() /= length(x.subRange(7,-1));

  ConstrainedProblem F=[&X](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x){
    phi.resize(5+X.d0);
    if(&tt){ tt.resize(5+X.d0); tt=ineqTT; }
    if(&J) {  J.resize(5+X.d0,11); J.setZero(); }
    if(&H) {  H.resize(11,11); H.setZero(); }

    //-- the scalar objective
    double a=x(0), b=x(1), c=x(2), r=x(3);
    phi(0) = a*b*c + 2.*r*(a*b + a*c +b*c) + 4./3.*r*r*r;
    if(&tt) tt(0) = fTT;
    if(&J){
      J(0,0) = b*c + 2.*r*(b+c);
      J(0,1) = a*c + 2.*r*(a+c);
      J(0,2) = a*b + 2.*r*(a+b);
      J(0,3) = 2.*(a*b + a*c +b*c) + 4.*r*r;
    }
    if(&H){
      H(0,1) = H(1,0) = c + 2.*r;
      H(0,2) = H(2,0) = b + 2.*r;
      H(0,3) = H(3,0) = 2.*(b+c);

      H(1,2) = H(2,1) = a + 2.*r;
      H(1,3) = H(3,1) = 2.*(a+c);

      H(2,3) = H(3,2) = 2.*(a+b);

      H(3,3) = 8.*r;
    }

    //-- positive
    double w=100.;
    phi(1) = -w*(a-.01);
    phi(2) = -w*(b-.01);
    phi(3) = -w*(c-.01);
    phi(4) = -w*(r-.01);
    if(&J){
      J(1,0) = -w;
      J(2,1) = -w;
      J(3,2) = -w;
      J(4,3) = -w;
    }

    //-- all constraints
    for(uint i=0;i<X.d0;i++){
      arr y, Jy;
      y=X[i];
      y.append(x);
      phi(i+5) = DistanceFunction_SSBox(Jy, NoArr, y);
//      Jy.subRange(3,5)() *= -1.;
      if(&J) J[i+5] = Jy.subRange(3,-1);
    }
  };

  checkJacobianCP(F, x, 1e-4);
  checkHessianCP(F, x, 1e-4);

  optConstrained(x, NoArr, F);

  checkJacobianCP(F, x, 1e-4);
  checkHessianCP(F, x, 1e-4);

  arr phi;
  F(phi, NoArr, NoArr, NoTermTypeA, x);
  cout <<"x=" <<x
        <<"\nsize = " <<x.subRange(0,3)
       <<"\ntransf = " <<x.subRange(4,-1)

      <<"\nphi=" <<phi <<endl;

  ors::Mesh points, box;
  points.V = X;
  box.setSSBox(2.*x(0), 2.*x(1), 2.*x(2), x(3));
  ors::Transformation t;
  t.pos.set( x.subRange(4,6) );
  t.rot.set( x.subRange(7,-1) );
  t.rot.normalize();
  t.applyOnPointArray(box.V);

  struct Transparent:GLDrawer{
    void glDraw(OpenGL &){ glColor(1,.5,.0,.5); }
  } trans;

  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(points);
  gl.add(trans);
  gl.add(box);
  gl.watch();

}

//===========================================================================

int MAIN(int argc, char** argv){
  testSCBoxFit();
  return 0;
}
