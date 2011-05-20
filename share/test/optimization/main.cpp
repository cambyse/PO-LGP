#include <MT/optimization.h>

struct MyProblem:public OptimizationProblem{
  arr C;
  uint n;
  
  void init(uint _n){
    n=_n;
    uint i,j;
    double condition=100.;
    //let C be a ortho-normal matrix (=random rotation matrix)
    C.resize(n,n);
    rndUniform(C,-1.,1.,false);
    for(i=0;i<n;i++){
      for(j=0;j<i;j++) C[i]()-=scalarProduct(C[i],C[j])*C[j];
      C[i]()/=norm(C[i]);
    }
    //let g be a diagonal with powers of the condition
    arr g(n,n);
    g.setZero();
    for(i=0;i<n;i++) g(i,i)=pow(condition, double(i) / double(n - 1));
    //the metric is equal C*g*C^T
    C=C*g*~C;
    cout <<C <<endl;
  }
  double f(arr *grad,const arr& x,int i=-1){
    CHECK(x.N==n,"");
    cout <<"  norm-x=" <<sumOfSqr(x) <<flush;
    double f=.5*scalarProduct(C,x,x);
    if(grad) (*grad)=C*x;
    return f;
  }
};

void testRprop(){
  MyProblem P;
  P.init(10);
  
  arr x(P.n);
  rndUniform(x,1.,10.,false);

  Rprop rprop;
  rprop.init(.01);
  rprop.loop(x,P,NULL,1e-5);
}

int main(int argn,char** argv){
  testRprop();
  return 0;
}