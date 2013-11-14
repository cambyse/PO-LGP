#include <Core/array.h>
#include <MT/MinSumGaussNewton.h>

void test(){
  struct MyProblem:MinSumGaussNewton{
    uint T,n;
    arr A,a;
    arr W;

    MyProblem(uint _T,uint _n){
      T=_T; n=_n;
      uint t;
      x.resize(T+1,n);
      A.resize(T+1,n,n);  a.resize(T+1,n);
      W.resize(n,n);
      rndUniform(x,0.,1.,false);
      for(t=0;t<=T;t++) A[t].setDiag(1.);
      W.setDiag(1.);
      A.setZero();
      a.setZero();
      //rndUniform(A,0.,1.,false);  for(t=0;t<=T;t++) A[t] = ~A[t]*A[t];
      //rndUniform(a,0.,1.,false);
      //rndUniform(W,0.,1.,false);  for(t=0;t<T;t++) W[t] = ~W[t]*W[t];
      a[T] = ARR(1.,1.);
      A[T].setDiag(1.);
      A[0].setDiag(1.);
      
      uint i;
      for(i=0;i<=T;i++){
        E.append(TUP(i,i));
        if(i>0) E.append(TUP(i-1,i));
        if(i<T) E.append(TUP(i+1,i));
      }
      E.reshape(E.N/2,2);
      del.resize(T+1);
      for(i=0;i<E.d0;i++) del(E(i,1)).append(i);
      cout <<"E=" <<E <<"del=" <<del <<endl;
    }
    double f(uint i,uint j,const arr& x_i,const arr& x_j){
      if(i==j)  return (~x_i*A[i]*x_i -2.*~a[i]*x_i)(0);
      arr d=x_j-x_i;
      return (~d*W*d)(0);
    }
    void reapproxPotentials(uint i,const arr& hat_x_i){
      uint k,m,n=hat_x_i.N;
      VERBOSE(2,cout <<"reapproximating potentials at node "<<i <<" at " <<hat_x_i <<endl);
      for(k=0;k<del(i).N;k++){
        m=del(i)(k);
        CHECK(E(m,1)==i,"");
        if(E(m,0)==i){ //node potential
          fij(m).A=A[i];
          fij(m).a=a[i];
          fij(m).hata=0.;
          fij(m).B.clear();  fij(m).C.clear();  fij(m).b.clear();
        }else{ //pair potential
          fij(m).A=W;
          fij(m).B=W;
          fij(m).C=-W;
          fij(m).a.resize(n);  fij(m).a.setZero();
          fij(m).b.resize(n);  fij(m).b.setZero();
          fij(m).hata=0.;
        }
      }
    }

  } f(3,2);
  
  f.go();
}

void test2(){
  struct MyProblem:MinSumGaussNewton{
    uint N;
    
    MyProblem(uint _N){
      N=_N;
      tolerance = 1e-3;
      x.resize(N,2);
      rndUniform(x,-1.,1.,false);
      
      uint i;
      for(i=0;i<N;i++){
        E.append(TUP(i,i));
        if(i>0) E.append(TUP(i-1,i));
        if(i<N-1) E.append(TUP(i+1,i));
      }
      E.reshape(E.N/2,2);
      del.resize(N);
      for(i=0;i<E.d0;i++) del(E(i,1)).append(i);
      cout <<"E=" <<E <<"del=" <<del <<endl;
    }

    void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){
      uint n=x_i.N;
      if(i==j){
        psi=ARR(0,0);  psiI.setDiag(0.,n);  psiJ.setDiag(0.,n);
        if(i==0){
          psi = x_i;
          psiI.setDiag(1.,n);
          psiJ.setDiag(0.,n);
        }
        if(i==N-1){
          psi  = ARR(cos(x_i(0)) + cos(x_i(1)), sin(x_i(0)) + sin(x_i(1)) - 1);
          psiI = ARR(-sin(x_i(0)),  -sin(x_i(1)), cos(x_i(0)), cos(x_i(1)));  psiI.reshape(n,n);
          psiJ.setDiag(0.,n);
        }
      }else{
        psi = x_i-x_j;
        psiI.setDiag( 1.,n);
        psiJ.setDiag(-1.,n);
      }
    }
  }f(5);
  
  f.go();
}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  MT::verboseLevel=1;
  
  test();
  //test2();

  return 0;
}
