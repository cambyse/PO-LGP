#include <MT/optimization.h>
#include <MT/functions.h>

#define DIM 1

struct ParticleAroundWalls:KOrderMarkovFunction {
  uint k;
  bool kern;
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);

  uint get_T(){ return 100; }
  uint get_k(){ return k; }
  uint get_n(){ return DIM; }
  uint get_m(uint t){
    uint T=get_T();
//    if(hasKernel()){ //the kernel is taking care of transition costs, phi only captures task
//      if(t==0 || t==T/2 || t==T/4 || t==3*T/4 || t==T-get_k()) return DIM;
//      return 0;
//    }
    if(t==0 || t==T/2 || t==T/4 || t==3*T/4 || t==T-get_k()) return 2*DIM;
    return DIM;
  }

  bool hasKernel(){ return kern; }
  double kernel(uint t0, uint t1){
    //if(t0==t1) return 1e3;
    return 1e0*::exp(-.0001*MT::sqr((double)t0-t1));
  }
};

void ParticleAroundWalls::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint T=get_T(), n=get_n(), k=get_k();

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T-k,"");

//  if(!hasKernel()){
    if(k==1)  phi = x_bar[1]-x_bar[0]; //penalize velocity
    if(k==2)  phi = x_bar[2]-2.*x_bar[1]+x_bar[0]; //penalize acceleration
    if(k==3)  phi = x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0]; //penalize jerk
//  }else{
    //phi only captures task costs
//  }

  //walls
  double eps=.1, power=2.;
  for(uint i=0;i<n;i++){ //add barrier costs to each dimension
    if(t==0)   phi.append(barrier(x_bar(0,i)+i+1., eps, power));    //first factor: ``lower than -i''
    if(t==T/4) phi.append(barrier(i+1.-x_bar(0,i), eps, power)); //middle factor: ``greater than i''
    if(t==T/2) phi.append(barrier(x_bar(k,i)+i+1., eps, power));    //last factor: ``lower than -i''
    if(t==3*T/4) phi.append(barrier(i+1.-x_bar(0,i), eps, power)); //middle factor: ``greater than i''
    if(t==T-k) phi.append(barrier(x_bar(k,i)+i+1., eps, power));    //last factor: ``lower than -i''
  }

  uint m=phi.N;
  CHECK(m==get_m(t),"");

  if(&J){ //we also need to return the Jacobian
    J.resize(m,k+1,n);
    J.setZero();

    //curvature
    for(uint i=0;i<n;i++){
//      if(!hasKernel()){
        if(k==1){ J(i,1,i) = 1.;  J(i,0,i) = -1.; }
        if(k==2){ J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
        if(k==3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
//      }
    }

    uint nn=n;
//    if(hasKernel()) nn=0;

    for(uint i=0;i<n;i++){ //add barrier costs to each dimension
      if(t==0)   J(nn+i,0,i) =  d_barrier(x_bar(0,i)+i+1., eps, power);
      if(t==T/4) J(nn+i,0,i) = -d_barrier(i+1.-x_bar(0,i), eps, power);
      if(t==T/2) J(nn+i,k,i) =  d_barrier(x_bar(k,i)+i+1., eps, power);
      if(t==3*T/4) J(nn+i,0,i) = -d_barrier(i+1.-x_bar(0,i), eps, power);
      if(t==T-k) J(nn+i,k,i) =  d_barrier(x_bar(k,i)+i+1., eps, power);
    }
  }
}
