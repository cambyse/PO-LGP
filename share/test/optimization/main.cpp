#include <MT/optimization.h>

struct SqrProblem:public ScalarFunction,VectorFunction{
  arr M,C;
  uint n;
  bool nonlinear;
  
  SqrProblem(uint _n, double condition=100.){
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
    nonlinear=false;
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
    if(!nonlinear){
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
  SqrProblem P(10);
  P.nonlinear=true;

  arr x(P.n),x0;
  rndUniform(x,1.,10.,false);
  x0=x;

  checkGradient((ScalarFunction&)P, x, 1e-3);
  checkGradient((VectorFunction&)P, x, 1e-3);
  
  optRprop(x, P, .01, NULL, 1e-5, 1000, 2);
  MT::wait();

  x=x0;
  optGradDescent(x, P, .01, NULL, 1e-5, 10000, -1., 2);
  MT::wait();

  x=x0;
  optGaussNewton(x, P, NULL, 1e-5, 1000, -1., 2);
  MT::wait();
}


struct ChainProblem:VectorChainFunction{
  uint T,n;
  arr A,a;
  arr Wi,Wj,w;

  ChainProblem(uint _T,uint _n){
    T=_T; n=_n;
    A.resize(T+1,n,n);  a.resize(T+1,n);
    Wi.resize(T,n,n);  Wj.resize(T,n,n);    w.resize(T+1,n);
    rndUniform(A,-1.,1.,false);
    rndUniform(a,-1.,1.,false);
    rndUniform(Wi,-1.,1.,false);
    rndUniform(Wj,-1.,1.,false);
    rndUniform(w,-1.,1.,false);
  }
  
  void fi(arr& y, arr* J, uint i, const arr& x_i){
    y = A[i]*(x_i - a[i]);
    if(J) *J = A[i];
  }
  void fij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j){
    y=Wi[j]*x_i-Wj[j]*x_j - w[j];
    if(Ji) *Ji =  Wi[j];
    if(Jj) *Jj = -Wj[j];
  }
};


void testDynamicProgramming(){
  ChainProblem P(10,3);
  
  arr x(P.T,P.n),x0;
  rndUniform(x,-1.,1.,false);
  x0=x;

  ConvertVector2SqrChainFunction PP(P);
  optDynamicProgramming(x, PP);
}



int main(int argn,char** argv){
  //testSqrProblem();
  testDynamicProgramming();
  return 0;
}