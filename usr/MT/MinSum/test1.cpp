//===========================================================================
//
// chain test
//

void testChain(){
  struct MyProblem:MinSumGaussNewton{
    uint T,n;
    arr A,a;
    arr Wi,Wj,w;

    MyProblem(uint _T,uint _n){
      T=_T; n=_n;
      tolerance = 1e-3;
      maxStep = 1.;
      x.resize(T+1,n);
      A.resize(T+1,n,n);  a.resize(T+1,n);
      Wi.resize(T,n,n);Wj.resize(T,n,n);    w.resize(T+1,n);
      rndUniform(x,0.,1.,false);
      rndUniform(A,-1.,1.,false);
      rndUniform(a,-1.,1.,false);
      rndUniform(Wi,-1.,1.,false);
      rndUniform(Wj,-1.,1.,false);
      rndUniform(w,-1.,1.,false);
      
      uint i;
      for(i=0;i<=T;i++){
        Msgs.append(TUP(i,i));
        if(i>0) Msgs.append(TUP(i-1,i));
        if(i<T) Msgs.append(TUP(i+1,i));
      }
      Msgs.reshape(Msgs.N/2,2);
      del.resize(T+1);
      for(i=0;i<Msgs.d0;i++) del(Msgs(i,1)).append(i);
    }
    
    void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){
      CHECK(j<=i,"");
      if(i==j){
        psi = A[i]*(x_i - a[i]);
        psiI = A[i];
        psiJ.clear();
      }else{
        psi=Wi[j]*x_i-Wj[j]*x_j - w[j];
        psiI= Wi[j];
        psiJ=-Wj[j];
      }
    }
    
#if 0
    double f(uint i,uint j,const arr& x_i,const arr& x_j){
      if(i==j)  return sumOfSqr(A[i]*(x_i - a[i]));
      return sumOfSqr(2.*x_j-x_i);
    }
#endif

#if 0
    void reapproxPotentials(uint i,const arr& x_i){
      uint j,k,m,n=x_i.N;
      arr psi,psiI,psiJ;
      VERBOSE(2,cout <<"reapproximating potentials at node "<<i <<" at " <<x_i <<endl);
      for(k=0;k<del(i).N;k++){
        m=del(i)(k);
        CHECK(Msgs(m,1)==i,"");
        j=Msgs(m,0);
        if(j<i) Psi(psi, psiI, psiJ, i, j, x_i, x[j]);
        else    Psi(psi, psiI, psiJ, j, i, x[j], x_i);
        if(j==i){ //node potential
          fij(m).A=~A[i]*A[i];
          fij(m).a=~A[i]*A[i]*a[i];
          fij(m).hata=sumOfSqr(A[i]*a[i]);
          fij(m).B.clear();  fij(m).C.clear();  fij(m).b.clear();
        }else{ //pair potential
          fij(m).A=~psiI*psiI;
          fij(m).B=~psiJ*psiJ;
          fij(m).C=~psiI*psiJ;
          fij(m).a.resize(n);  fij(m).a.setZero();
          fij(m).b.resize(n);  fij(m).b.setZero();
          fij(m).hata=0.;
        }
      }
    }
#endif
  } f(3,2);
  
  f.init();
  f.step(5);
}



//===========================================================================
//
// chain test
//

void test2(){
  struct MyProblem:MinSumGaussNewton{
    uint N;
    
    MyProblem(uint _N){
      N=_N;
      tolerance = 1e-3;
      maxStep = 1.;
      x.resize(N,2);
      rndUniform(x,-1.,1.,false);

      //2D state space, trajectory with N steps
      uint i;
      for(i=0;i<N;i++){
        Msgs.append(TUP(i,i));
        if(i>0) Msgs.append(TUP(i-1,i));
        if(i<N-1) Msgs.append(TUP(i+1,i));
      }
      Msgs.reshape(Msgs.N/2,2);
      del.resize(N);
      for(i=0;i<Msgs.d0;i++) del(Msgs(i,1)).append(i);
      cout <<"Msgs=" <<Msgs <<"del=" <<del <<endl;
    }

    void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){
      uint n=x_i.N;
      if(i==j){
        psi=ARR(0,0);  psiI.setDiag(0.,n);  psiJ.setDiag(0.,n); //default node potential: zero
        if(i==0){ //node potential: start node: condition on zero position
          psi = x_i;
          psiI.setDiag(1.,n);
        }
        if(i==N-1){ //node potential: final node: condition on 2-link arm effector to be at (0,1) (I think)
          psi  = ARR( cos(x_i(0)) + cos(x_i(1)), sin(x_i(0)) + sin(x_i(1)) - 1);
          psiI = ARR(-sin(x_i(0)), -sin(x_i(1)), cos(x_i(0)), cos(x_i(1)));  psiI.reshape(n,n);
        }
      }else{ //transition potentials: just difference
        psi = x_i-x_j;
        psiI.setDiag( 1.,n);
        psiJ.setDiag(-1.,n);
      }
    }
  }f(5);

  f.init();
  f.step(20);
}

