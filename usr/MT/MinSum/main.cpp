#include <MT/array.h>
#include <MT/MinSumGaussNewton.h>
#include <MT/util.h>

#include "test1.cpp" //other tests

//===========================================================================
//
// two competing 1D potentials
//

void testDifficultPair(){
  struct MyProblem:MinSumGaussNewton{
    MyProblem(){
      tolerance = 1e-5;
      maxStep = 1.;
      x.resize(2,1);
      x.setZero();
      
      Msgs.append(TUP(0,0));
      Msgs.append(TUP(0,1));
      Msgs.append(TUP(1,0));
      Msgs.append(TUP(1,1));
      Msgs.reshape(Msgs.N/2,2);
      del.resize(2);
      for(uint i=0;i<Msgs.d0;i++) del(Msgs(i,1)).append(i);
    }

    void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){
      if(i==j){
        if(i==0){
          psi=ARR(pow(x_i(0)+2,4.));
          psiI.setDiag(4*pow(x_i(0)+2,3.),1);
        }else{
          psi=ARR(pow(x_i(0)-2.,4.));
          psiI.setDiag(4*pow(x_i(0)-2.,3.),1);
        }
        psiJ.clear();
      }else{
        double prec=1e2;
        psi = prec*(x_i - x_j);
        psiI.setDiag(prec,1);
        psiJ.setDiag(-prec,1);
      }
    }
  }f;

  f.init();
  f.step(10);
}


//===========================================================================
//
// random network of competing 1D potentials
//

struct RndNetProblem:public MinSumGaussNewton{
  uintA E;
  arr f_center;
  
  RndNetProblem(uint n,uint degree){
    //generate random graph
    //graphRandomFixedDegree(E, n, degree);
    graphRandomTree(E, n);
    //cout <<E <<endl;
    setUndirectedGraph(n, E);
    //cout <<Msgs <<endl;
    
    //x-initialization
    x.resize(n,1);
    x.setZero();
    
    //potential randomization
    f_center.resize(n);
    for(uint i=0;i<n;i++) f_center(i)=rnd.uni(-2.,2.);
    
    //parameters
    tolerance = 1e-5;
    maxStep = 1.;
    damping=1.;
  }
  
  void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){
    if(i==j){
      double d=x_i(0)-f_center(i);
      //psi=ARR(pow(d,4.));  psiI.setDiag(4*pow(d,3.),1);
      psi=ARR(d);  psiI.setDiag(1.,1);
      psiJ.clear();
    }else{
      double prec=1e2;
      psi = prec*(x_i - x_j);
      psiI.setDiag(prec,1);
      psiJ.setDiag(-prec,1);
    }
  }
};

void testRndNet(){
  RndNetProblem f(4,3);
  
  f.init();
  //f.damping=0.;
  f.step(50);
}



int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  
  //testChain();
  //test2();
  //testDifficultPair();
  testRndNet();
  
  return 0;
}
