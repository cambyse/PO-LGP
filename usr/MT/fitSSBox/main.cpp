#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>

#include <Optim/optimization.h>


//===========================================================================

void TEST(SCBoxFit){
  arr X(1,3);
  rndUniform(X, -1., 1.);

  arr x(14);
  rndUniform(x, .1, 1.);

  ConstrainedProblem F=[&X](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x){
    phi.resize(5+X.d0);
    if(&tt){ tt.resize(5+X.d0); tt=ineqTT; }
    if(&J) {  J.resize(5+X.d0,14); J.setZero(); }
    if(&H) {  H.resize(14,14); H.setZero(); }

    //-- the scalar objective
    double a=x(3), b=x(4), c=x(5), r=x(6);
    phi(0) = a*b*c + 2.*r*(a*b + a*c +b*c) + 4./3.*r*r*r;
    if(&tt) tt(0) = fTT;
    if(&J){
      J(0,3) = b*c + 2.*r*(b+c);
      J(0,4) = a*c + 2.*r*(a+c);
      J(0,5) = a*b + 2.*r*(a+b);
      J(0,6) = 2.*(a*b + a*c +b*c) + 4.*r*r;
    }
    if(&H){
      H(3,4) = H(4,3) = c + 2.*r;
      H(3,5) = H(5,3) = b + 2.*r;
      H(3,6) = H(6,3) = 2.*(b+c);

      H(4,5) = H(5,4) = a + 2.*r;
      H(4,6) = H(6,4) = 2.*(a+c);

      H(5,6) = H(6,5) = 2.*(a+b);

      H(6,6) = 8.*r;
    }

    //-- positive
    double w=100.;
    phi(1) = -w*a+.01;
    phi(2) = -w*b+.01;
    phi(3) = -w*c+.01;
    phi(4) = -w*r+.01;
    if(&J){
      J(1,3) = -w;
      J(2,4) = -w;
      J(3,5) = -w;
      J(4,6) = -w;
    }

    //-- all constraints
    for(uint i=0;i<X.d0;i++){
      phi(i+5) = DistanceFunction_SSBox(&J?J[i+5]():NoArr, NoArr, x);
    }
  };

  checkJacobianCP(F, x, 1e-4);
  checkHessianCP(F, x, 1e-4);

  optConstrained(x, NoArr, F, OPT(verbose=4, damping=1.));
}

//===========================================================================

int MAIN(int argc, char** argv){
  testSCBoxFit();
  return 1;
}
