#include <Core/array.h>

struct Message{
  double abs;
  arr A,a;
  double eval(arr& x){ return (~x*A*x - 2.*~a*x + abs).scalar(); }
};


/*struct ProblemAbstraction{
  virtual void getPhi(arr& Phi, arr& PhiJ, uint i, const arr& x_i) = 0;
  virtual void getPsi(arr& Psi, arr& PsiA, arr& PsiB, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
  virtual void getW  (arr& W, uint i, uint j, const arr& x_i, const arr& x_j) = 0;

  double costAndGradient(arr* grad,const arr& q);
};*/

struct Fi{  arr A,a;       double hata; };
struct Fij{ arr A,B,C,a,b; double hata; };


struct MinSumGaussNewton{
  arr x;
  double tolerance;

  uintA Msgs; //edges
  MT::Array<uintA> del; //in-neighbors
  MT::Array<Message> mu;

  MT::Array<Fi>  fi;
  MT::Array<Fij> fij;

  virtual double phi(uint i,const arr& x_i) = 0;
  virtual double psi(uint i,uint j,const arr& x_i,const arr& x_j) = 0;
  virtual void reapproxPotentials(uint i,const arr& hat_x_i) = 0;

  double totalCost();
  void go();
};

double MinSumGaussNewton::totalCost(){
  double F=0.;
  uint m,i,j;
  for(m=0;m<Msgs.d0;m++){
    i=Msgs(m,0);  j=Msgs(m,1);
    if(i==j) F += phi(i,x[i]);
    else if(i<j) F += psi(i,j,x[i],x[j]);
    //the case i>j is excluded to not count couplings twice
  }
  return F;
}

void MinSumGaussNewton::go(){ 
  uint N=x.d0,n=x.d1;
  uint i,j,k,m;
  double fx,fy;
  arr A,a,Delta,y;
  double alpha=1.;

  //init messages zero
  mu.resize(E.d0);
  for(m=0;m<mu.N;m++){
    mu(m).a.resize(x.d1);      mu(m).a.setZero();
    mu(m).A.resize(x.d1,x.d1); mu(m).A.setDiag(1e-6);
    i=Msgs(m,0); j=Msgs(m,1);
    if(i==j)     mu(m).abs=0.; //phi(i,x[i]);
    else if(i<j) mu(m).abs=0.; //psi(i,j,x[i],x[j]);
    else         mu(m).abs=0.; //psi(j,i,x[j],x[i]);
  }

  //iterate
  bool forward=true;
  i=0;
  for(uint bla=0;bla<20;bla++){ //iterate over nodes

    cout <<"Iteration " <<bla <<" F=" <<totalCost() <<endl;

    for(;;){ //iterate optimizing at node i
      reapproxPotentials(i,x[i]);
      fx=0.;
      A.resize(n,n);  A.setZero();
      a.resize(n);    a.setZero();
      for(k=0;k<del(i).N;k++){
        m=del(i)(k);
        fx += mu(m).abs + (~x[i]*mu(m).A*x[i] -2.*~mu(m).a*x[i])(0);
        A  += mu(m).A;
        a  += mu(m).a;
      }
      Delta = lapack_Ainv_b_sym(A, a);
      Delta -= x[i];
      VERBOSE(2,cout <<"optimizing over node "<<i <<": x=" <<x[i] <<" f(x)=" <<fx <<" Delta=" <<Delta <<endl);

      //x[i]() += Delta;  break;   VERBOSE(2,cout <<" - FORCE" <<endl);
      
      //stopping criterion
      if(norm(Delta)<tolerance /*|| evals>maxEvals*/) break;
      
      for(;;){ //iterate over step sizes
        y = x[i] + alpha*Delta;
        fy=0.;
        reapproxPotentials(i,y);
        for(k=0;k<del(i).N;k++){
          m=del(i)(k);
          fy += mu(m).abs + (~y*mu(m).A*y -2.*~mu(m).a*y)(0);
        }
        VERBOSE(2,cout /*<<evals*/ <<" \tprobing y=" <<y <<" \tf(y)=" <<fy <<" \t|Delta|=" <<norm(Delta) <<" \talpha="<< alpha <<std::flush);
        CHECK_EQ(fy,fy,"cost seems to be NAN: f(y)=" <<fy);
        if(fy <= fx) break;
        //if(evals>maxEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
        //decrease stepsize
        alpha = .5*alpha;
        VERBOSE(2,cout <<" - reject" <<endl);
      }
      VERBOSE(2,cout <<" - ADOPT" <<endl);
    
      //adopt new point and adapt stepsize
      x[i] = y;
      fx = fy;
      alpha  = pow(alpha,0.5);
    }

    if(forward){ i++; if(i==N-1) forward=false; }
    else{ i--; if(!i) forward=true; }
  }


}

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
      W.resize(T,n,n);
      rndUniform(x,0.,1.,false);
      for(t=0;t<=T;t++) A[t].setDiag(1.);
      for(t=0;t<T;t++) W[t].setDiag(1.);
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
        Msgs.append(TUP(i,i));
        if(i>0) Msgs.append(TUP(i-1,i));
        if(i<T) Msgs.append(TUP(i+1,i));
      }
      Msgs.reshape(E.N/2,2);
      del.resize(T+1);
      for(i=0;i<E.d0;i++) del(Msgs(i,1)).append(i);
      cout <<"E=" <<Msgs <<"del=" <<del <<endl;
    }
    double phi(uint i,const arr& x){
      //cout <<A[i] <<a[i] <<x <<endl;
      return (~x*A[i]*x -2.*~a[i]*x)(0);
    }
    double psi(uint i,uint j,const arr& x_i,const arr& x_j){
      //cout <<W[i] <<x_i <<x_j <<endl;
      arr d=x_j-x_i;
      return (~d*W[i]*d)(0);
    }
    void reapproxPotentials(uint i,const arr& hat_x_i){
      uint j,k,l,m,mm;
      double abs_sum;
      arr A_sum,a_sum,tmp;
      VERBOSE(2,cout <<"reapproximating all messages for node "<<i <<endl);
      for(k=0;k<del(i).N;k++){
        m=del(i)(k);
        CHECK_EQ(Msgs(m,1),i,"");  j=Msgs(m,0);
        VERBOSE(3,cout <<"  reapproximating message "<<m <<":" <<j <<"->" <<i <<endl);
        if(j==i){ //node potential
          mu(m).A=A[i];
          mu(m).a=a[i];
          mu(m).abs=0.; //phi(i,hat_x_i);
        }else{
          A_sum.resize(n,n);  A_sum.setZero();
          a_sum.resize(n);    a_sum.setZero();
          abs_sum=0.;
          for(l=0;l<del(j).N;l++){
            mm=del(j)(l);
            CHECK_EQ(Msgs(mm,1),j,"");
            if(E(mm,0)==i) continue; //(exclude i->j)
            VERBOSE(3,cout <<"    collecting message "<<mm <<":" <<Msgs(mm,0) <<"->" <<j <<endl);
            abs_sum += mu(mm).abs;
            A_sum   += mu(mm).A;
            a_sum   += mu(mm).a;
          }
          VERBOSE(4,cout <<"  A_sum=" <<A_sum <<"a_sum=" <<a_sum <<"abs_sum=" <<abs_sum <<endl);
          inverse_SymPosDef(tmp,A_sum);
          mu(m).a = tmp * a_sum;
          tmp += W[i<j?i:j];
          inverse_SymPosDef(mu(m).A, tmp);
          mu(m).a = mu(m).A * mu(m).a;
          inverse_SymPosDef(tmp,A_sum+W[i<j?i:j]); //THIS IS INEFFICIENT CAN BE SIMPLIFIED
          mu(m).abs = abs_sum - (~a_sum * tmp * a_sum)(0); //+ psi(i,j,hat_x_i,x[j]); 
        }
        VERBOSE(4,cout <<"  new message = " <<mu(m).A <<mu(m).a <<mu(m).abs <<endl);
      }
    }
  } f(4,2);
  
  f.go();

}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  MT::verboseLevel=1;
  
  test();

  return 0;
}
