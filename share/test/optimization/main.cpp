#include <MT/optimization.h>

struct SqrProblem:public ScalarFunction,VectorFunction{
  arr M,C;
  uint n;
  bool nonlin;
  
  void init(uint _n, double condition=100.){
    n=_n;
    uint i,j;
    //let M be a ortho-normal matrix (=random rotation matrix)
    M.resize(n,n);
    rndUniform(M,-1.,1.,false);
    for(i=0;i<n;i++){
      for(j=0;j<i;j++) M[i]()-=scalarProduct(M[i],M[j])*M[j];
      M[i]()/=norm(M[i]);
    }
    //we condition each column of M with powers of the condition
    for(i=0;i<n;i++) M[i]() *= pow(condition, double(i) / (2.*double(n - 1)));
    //the metric is equal M*M^T
    C=~M*M;
    //arr U,d,V;    svd(U, d, V, C);    cout <<U <<d <<V <<M <<C <<endl;
    nonlin=true;
  }
  
#if 0
  double fs(arr *grad,const arr& x){
    CHECK(x.N==n,"");
    double f=scalarProduct(C,x,x);
    if(grad) (*grad)=2.*C*x;
    if(nonlin){
      //double s=1e7;
      if(grad) (*grad) *= 1./(f+1.);
      f = log(f+1.);
    }
    return f;
  }
#else
  double fs(arr *grad,const arr& x){
    arr y;
    fv(y, grad, x);
    if(grad) *grad=2.*~y*(*grad);
    return sumOfSqr(y);
  }
#endif

  void fv(arr& y, arr *J,const arr& x){
    CHECK(x.N==n,"");
    if(!nonlin){
      y = M*x;
      if(J) (*J)=M;
    }else{
      arr xx=atan(x);
      y=M*xx;
      if(J){
        arr gg(xx.N);
        for(uint i=0;i<gg.N;i++) gg(i) = 1./(1.+x(i)*x(i));
        *J = M*diag(gg);
      }
    }
  }
};

void testSqrProblem(){
  SqrProblem P;
  P.init(10);
  
  arr x(P.n),x0;
  rndUniform(x,1.,10.,false);
  x0=x;

  checkGradient((ScalarFunction&)P, x, 1e-3);
  checkGradient((VectorFunction&)P, x, 1e-3);
  
  optRprop(x, P, .01, NULL, 1e-5, 1000, 2);
  MT::wait();

  MT::verboseLevel=2;
  x=x0;
  optGaussNewton(x, P, NULL, 1e-5, 1000, -1., 2);
  MT::wait();
}




int main(int argn,char** argv){
  testSqrProblem();
  return 0;
}