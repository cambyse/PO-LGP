#include <MT/kOrderMarkovProblem.h>
#include <MT/functions.h>

#define DIM 3

struct ParticleAroundWalls:KOrderMarkovFunction {
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);

  uint get_T(){ return 100; }
  uint get_k(){ return 3; }
  uint get_n(){ return DIM; }
  uint get_m(uint t){
    uint T=get_T();
    if(t==0 || t==T/2 || t==T-get_k()) return 2*DIM;
    return DIM;
  }
};

void ParticleAroundWalls::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint T=get_T(), n=get_n(), k=get_k();

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T-k,"");

  if(k==1)  phi = x_bar[1]-x_bar[0]; //penalize velocity
  if(k==2)  phi = x_bar[2]-2.*x_bar[1]+x_bar[0]; //penalize acceleration
  if(k==3)  phi = x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0]; //penalize jerk
    
  double eps=.1, power=2.;
  for(uint i=0;i<n;i++){ //add barrier costs to each dimension
    if(t==0)   phi.append(barrier(x_bar(0,i)+i+1., eps, power));    //first factor: ``lower than -i''
    if(t==T/2) phi.append(barrier(i+1.-x_bar(0,i), eps, power)); //middle factor: ``greater than i''
    if(t==T-k) phi.append(barrier(x_bar(k,i)+i+1., eps, power));    //last factor: ``lower than -i''
  }

  uint m=phi.N;
  CHECK(m==get_m(t),"");

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
      if(t==0)   J(n+i,0,i) =  d_barrier(x_bar(0,i)+i+1., eps, power);
      if(t==T/2) J(n+i,0,i) = -d_barrier(i+1.-x_bar(0,i), eps, power);
      if(t==T-k) J(n+i,k,i) =  d_barrier(x_bar(k,i)+i+1., eps, power);
    }
  }
}
