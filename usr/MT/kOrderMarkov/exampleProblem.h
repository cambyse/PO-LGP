#include "kOrderMarkovProblem.h"

struct ParticleAroundWalls:KOrderMarkovFunction {
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);

  uint get_T(){ return 100; }
  uint get_k(){ return 2; }
  uint get_n(){ return 1; }
  uint get_m(uint t){ return 2; }
};

void ParticleAroundWalls::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint T=get_T(), n=get_n(), k=get_k(), m=get_m(t);

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T-k,"");

  // the three coupled states
  arr x0=x_bar[0];
  arr x1=x_bar[1];
  arr x2=x_bar[2];

  phi.resize(m);
  phi.setZero();

  phi(0) = scalar(x1 - .5*(x2+x0)); //penalize curvature of path

  //phi(1) contains the ``task costs''
  if(t==0)   phi(1) = scalar(x0);    //first factor: ``tying to zero''
  if(t==T-k) phi(1) = scalar(x2);    //last factor: ``tying to zero''
  if(t==T/2) phi(1) = scalar(x0-1.); //middle factor: ``tying to 1.''

  if(&J){ //we also need to return the Jacobian
    J.resize(m,k+1,n);
    J.setZero();

    CHECK(n==1,"only implemented for n=1 yet");

    //curvature
    J[0][1] = 1.;
    J[0][0] = -.5;
    J[0][2] = -.5;

    if(t==0)   J[1][0] = 1.;
    if(t==T-k) J[1][2] = 1.;
    if(t==T/2) J[1][0] = 1.;
  }
}
