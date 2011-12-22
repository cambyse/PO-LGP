#include "MT/optimization.h"
#include "MT/array.h"
#include "MT/util.h"
#include <DZ/WritheMatrix.h> 

struct WritheSegmentTest:public OptimizationProblem {
  ors::Vector c,d;
  arr Jq;

  WritheSegmentTest(){
    c.setRandom();
    d.setRandom();
    Jq.resize(6,6);
    rndUniform(Jq,-1.,1.,false); 
  }
  double f(arr* J, const arr& q,int i=-1){
    CHECK(q.nd==1 && q.N==6,"");
    arr x = Jq * q; //we assume the point positions are a random transformation from q parameters
    ors::Vector a,b;
    a.set(&x(0));
    b.set(&x(3));
    double y; GetWritheSegment(y,a,b,c,d);
    if(J){ //the function wants also the Jacobian at this point
     Jq.reshape(2,3,q.N);
     GetWritheJacobianSegment(*J, a, b, c, d, Jq);
     Jq.reshape(6,6);
      J->reshape(2,3);
    } 
    return y;
  }
 
};

struct WritheMatrixTest:public OptimizationProblem{
  uint N,n;
  arr rope2;
  arr Jq;
  arr one_point_jacobian;
  WritheMatrixTest(uint _N, uint _n){
    N=_N;
    n=_n;
    rope2.resize(N+1,3); // rope = segments+1
    rndUniform(rope2,-1.,1.,false);
    Jq.resize(N*3,n);// rope = segments+1
    rndUniform(Jq,-1.,1.,false);
    one_point_jacobian = zeros(3,n);  // We need this jacobian for the first joint in chain which is usually fixed J == 0 !!
      
  }

  void F(arr& y, arr* J, const arr& q, int i=-1){ //TODO check EVERYTHING!!!
    arr NewJq = one_point_jacobian; NewJq.append(Jq); // trick for one more point in rope 
    arr rope1 = NewJq * q; //we assume the rope is a random projection of the parameters q
    rope1.reshape(N+1,3); // rope = segments+1
    GetWritheMatrix(y, rope1, rope2, N);
    if(J){ //the function wants also the Jacobian at this point
      //Jq.reshape(N,3,n);
      WritheJacobian(*J, rope1, rope2, Jq,N);
      Jq.reshape(N*3,n);
      J->reshape(N*N,n); 
    }
  }
};


struct WritheScalarTest:public OptimizationProblem{
  uint N,n;
  arr rope2;
  arr Jq;
  arr one_point_jacobian;
  WritheScalarTest(uint _N, uint _n){
    N=_N;
    n=_n;
    rope2.resize(N+1,3); // rope = segments+1
    rndUniform(rope2,-1.,1.,false);
    Jq.resize(N*3,n);// rope = segments+1
    rndUniform(Jq,-1.,1.,false);
    one_point_jacobian = zeros(3,n);  // We need this jacobian for the first joint in chain which is usually fixed J == 0 !!
      
  }

  void F(arr& y, arr* J, const arr& q, int i=-1){ //TODO check EVERYTHING!!!
    arr NewJq = one_point_jacobian; NewJq.append(Jq); // trick for one more point in rope 
    arr rope1 = NewJq * q; //we assume the rope is a random projection of the parameters q
    rope1.reshape(N+1,3); // rope = segments+1
    GetScalarWrithe(y, rope1, rope2, N);
    if(J){ //the function wants also the Jacobian at this point
      //Jq.reshape(N,3,n);
     ScalarJacobian(*J, rope1, rope2, Jq,N);
    // cout << "Ja"<<J<<endl;
      Jq.reshape(N*3,n);
      cout  <<"Jacobian"<<*J<<endl;
     // J->reshape(n,n); 
    }
  }
};

void WritheGradientCheck(){
  //check WritheSegment
  WritheSegmentTest fs = WritheSegmentTest();
  arr x(2*3);
  for(uint k=0;k<1;k++){
    rndUniform(x,-1.,1.,false); //test the gradient for a random rope1
    checkGradient(fs, x, 1e-4);
  }


  //check WritheMatrix
  uint N=10, n=10;  
  WritheMatrixTest fm =  WritheMatrixTest(N,n);
  x.resize(n);
  for(uint k=0;k<100;k++){
    rndUniform(x,-1.,1.,false); //test the gradient for a random rope1
    checkGradient_vec(fm, x, 1e-4);
  }
}

void GradientScalarCheck(){
 
  //check WritheMatrix
  uint N=10, n=7;
  WritheScalarTest ws =  WritheScalarTest(N,n);
  arr x;
  x.resize(n);
  for(uint k=0;k<1000;k++){
    rndUniform(x,-1.,1.,false); //test the gradient for a random rope1
    checkGradient_vec(ws, x, 1e-4);
  }
}
