#include <stdlib.h>
#include <Kin/roboticsCourse.h>
#include <Gui/opengl.h>
#include <Algo/kalman.h>
#include <Motion/motion.h>

#include <Hardware/racer/racer.h>

void testBenchmark(){
  arr k = ARR(0, 3.1623,   8.6762,   2.6463,   2.0126);

  RacerBalancingBenchmark.exploration=.1;
  RacerBalancingBenchmark.display=true;
  RacerBalancingBenchmark.fs(NoArr, NoArr, k);
  RacerBalancingBenchmark.display=false;
}

void testGradient(){
  arr k = ARR(0, 3.1623,   8.6762,   2.6463,   2.0126);

  //  RacerBalancingBenchmark.display=true;
  RacerBalancingBenchmark.T=30;
  RacerBalancingBenchmark.exploration=.1;
//  RacerBalancingBenchmark.noise=.1;

  cout <<"********* FD " <<endl;
  RacerBalancingBenchmark.exploration=.0;
  arr g_true = zeros(k.N);
  uint M=100;
  for(uint m=0;m<M;m++){
    RacerBalancingBenchmark.fixedRandomSeed=m;
    g_true += finiteDifferenceGradient(RacerBalancingBenchmark, k);
    cout <<g_true/double(m+1) <<endl;
  }
  g_true /= double(M);

  cout <<"********* Regresion " <<endl;
  RacerBalancingBenchmark.exploration=.0;
  RacerBalancingBenchmark.fixedRandomSeed=-1;
  M=100;
  arr X = .1 * randn(M,k.N);
  arr y = zeros(M);
  for(uint m=0;m<M;m++){
    y(m) = RacerBalancingBenchmark.fs(NoArr, NoArr, k+X[m]);
    arr Xsub = catCol(ones(m+1),X({0,m}));
    arr ysub = y({0,m});
    cout <<inverse_SymPosDef(~Xsub*Xsub + 1e-6*eye(k.N+1))* ~Xsub * ysub <<endl;
  }
  X = catCol(ones(M),X);
  arr b = inverse_SymPosDef(~X* X)* ~X * y;

  cout <<"********* PG " <<endl;
  RacerBalancingBenchmark.fixedRandomSeed=-1;
  RacerBalancingBenchmark.exploration=.1;

//  return;

  M=1000;
  arr g = zeros(k.N);
  arr R = zeros(M);
  arr d_log_mu = zeros(M, k.N);
  for(uint m=0;m<M;m++){
    R(m) = RacerBalancingBenchmark.fs(d_log_mu[m](), NoArr, k);
//    cout <<(~R*d_log_mu)/double(m+1) <<endl;
  }

  for(uint i=0;i<k.N;i++){
    double up=0., dn=0.;
    for(uint m=0;m<M;m++){
      up += mlr::sqr(d_log_mu(m,i)) * R(m);
      dn += mlr::sqr(d_log_mu(m,i));
    }
    double b = up/dn;

    for(uint m=0;m<M;m++){
      g(i) += d_log_mu(m,i) * (R(m) - b);
    }
  }
  g /= double(M);


  cout <<"vanilla grad :" <<endl <<g <<endl;

  cout <<"finite difference grad:" <<endl <<g_true <<endl;

  cout <<"regression grad:" <<endl <<b <<endl;
}


int main(int argc,char **argv){
//  testBenchmark();
  testGradient();

  return 0;
}
