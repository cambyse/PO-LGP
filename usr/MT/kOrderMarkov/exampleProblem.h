#include "kOrderMarkovProblem.h"
#include <MT/functions.h>

#define DIM 3

struct ParticleAroundWalls:KOrderMarkovFunction {
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);

  uint get_T(){ return 100; }
  uint get_k(){ return 3; }
  uint get_n(){ return DIM; }
  uint get_m(uint t){ return 2*DIM; }
};

void ParticleAroundWalls::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint T=get_T(), n=get_n(), k=get_k(), m=get_m(t);

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T-k,"");

  phi.resize(m);
  phi.setZero();

  if(k==1)  phi.setVectorBlock(x_bar[1]-x_bar[0], 0); //penalize velocity
  if(k==2)  phi.setVectorBlock(x_bar[2]-2.*x_bar[1]+x_bar[0], 0); //penalize acceleration
  if(k==3)  phi.setVectorBlock(x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0], 0); //penalize jerk
    
  double eps=.1, power=2.;
  //phi(1) contains the ``task costs''
  for(uint i=0;i<n;i++){ //add barrier costs to each dimension
    if(t==0)   phi(n+i) = barrier(x_bar(0,i)+i, eps, power);    //first factor: ``tying to zero''
    if(t==T-k) phi(n+i) = barrier(x_bar(k,i)+i, eps, power);    //last factor: ``tying to zero''
    if(t==T/2) phi(n+i) = barrier(1.+i-x_bar(0,i), eps, power); //middle factor: ``tying to 1.''
  }
  if(&J){ //we also need to return the Jacobian
    J.resize(m,k+1,n);
    J.setZero();

    //curvature
    for(uint i=0;i<n;i++){
      if(k==1){ J(i,1,i) = 1.;  J(i,0,i) = -1.; }
      if(k==2){ J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
      if(k==3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
    }

    for(uint i=0;i<n;i++){ //add barrier costs to each dimension
      if(t==0)   J(n+i,0,i) = d_barrier(x_bar(0,i)+i, eps, power);
      if(t==T-k) J(n+i,k,i) = d_barrier(x_bar(k,i)+i, eps, power);
      if(t==T/2) J(n+i,0,i) = -d_barrier(1.+i-x_bar(0,i), eps, power);
    }
  }
}
