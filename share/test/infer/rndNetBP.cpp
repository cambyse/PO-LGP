
 
void randomPairNet(infer::VariableList& vars,infer::FactorList &facs,uint N,uint n,double connectivity){
  uint i,j;
  
  //-- generate RVs
  vars.resize(N);
  for(i=0;i<N;i++) vars(i) = new infer::Variable(n, STRING("var("<<i<<")"));

  //-- generate coupling factors
  infer::Factor *f;
  for(i=0;i<N;i++) for(j=i+1;j<N;j++){
    if(rnd.uni()<connectivity){
      f = new infer::Factor(ARRAY(vars(i), vars(j)));
      //f -> setRandom();
      f -> setText("[ .75 .25 ; .25 .75]");
      //F(i,j) -> setText("[ 1 0 ; 0 1]");
      facs.append(f);
    }
  }

  //connectThemUp(vars,facs);
  
  //-- generate evidence factors
  infer::Factor evid;
  infer::Variable *v;
  for_list(i,v,vars) if(v->factors.N){
    f=v->factors(0);
    evid.init(ARRAY(v));
    evid.setText("[.2 .8]");
    tensorMultiply(*f,evid);
    break;
  }
}

      
void rndNetBPwithExcludeEchoMessages(uint N,uint n,double connectivity,uint order){
  uint i,j,k,l,ll;
  
  //-- generate RVs
  infer::VariableList V(N);
  for(i=0;i<N;i++) V(i) = new infer::Variable(n, STRING("var("<<i<<")"));

  //-- generate coupling factors
  infer::FactorList F(N,N);  F.memMove=true;  F.setZero();
  for(i=0;i<N;i++) for(j=i+1;j<N;j++){
    if(rnd.uni()<connectivity){
      F(i,j) = new infer::Factor(ARRAY(V(i), V(j)));
      //F(i,j) -> setRandom();
      F(i,j) -> setText("[ .75 .25 ; .25 .75]");
      //F(i,j) -> setText("[ 1 0 ; 0 1]");
    }
  }

  //-- generate evidence factors
  infer::FactorList E(N);  E.memMove=true;  E.setZero();
  for(i=0;i<N;i++){
    if(!i) E(i) = new infer::Factor(ARRAY(V(i)),"[.2 .8]");
    else   E(i) = new infer::Factor(ARRAY(V(i)));
  }

  //cout <<"** list of all factors = ";  listWrite(F,cout,"\n  ");  listWrite(E,cout,"\n  ");   cout <<endl;

  //-- exact solution
  infer::Factor f,f_elim,f_diff;
  infer::FactorList facs;
  for(i=0;i<F.N;i++) if(F.elem(i)) facs.append(F.elem(i));
  for(i=0;i<E.N;i++) if(E.elem(i)) facs.append(E.elem(i));
  arr ex_post(N),post(N);
  for(i=0;i<N;i++){
    eliminationAlgorithm(f,facs,ARRAY(V(i)));
    //cout <<"exact marginal of V("<<i<<")=" <<f <<endl;
    ex_post(i) = f.P(0);
  }
  
  for(i=0;i<N;i++) for(j=i+1;j<N;j++) F(j,i) = F(i,j);
  
  //-- generate messages
  infer::FactorList M(N,N);
  for(i=0;i<N;i++) for(j=i+1;j<N;j++) if(F(i,j)){
    M(i,j) = new infer::Factor(ARRAY(V(j)));
    M(j,i) = new infer::Factor(ARRAY(V(i)));
  }
  infer::FactorList M1(N,N,N),M2; M2.resize(ARRAY(N,N,N,N));
  for(i=0;i<N;i++) for(j=i+1;j<N;j++) if(F(i,j)){
    for(l=0;l<N;l++){
      M1(i,j,l) = new infer::Factor(ARRAY(V(j)));
      M1(j,i,l) = new infer::Factor(ARRAY(V(i)));
    }
    for(l=0;l<N;l++) for(ll=l+1;ll<N;ll++){
	M2(ARRAY(i,j,l,ll)) = new infer::Factor(ARRAY(V(j)));
	M2(ARRAY(j,i,l,ll)) = new infer::Factor(ARRAY(V(i)));
	M2(ARRAY(i,j,ll,l)) = M2(ARRAY(i,j,l,ll));
	M2(ARRAY(j,i,ll,l)) = M2(ARRAY(j,i,l,ll));
    }
  }
  
  MT::Array<infer::Factor> marg(N);
  uint t;
  for(t=0;t<10;t++){ //LOOP
    if(order==0){
    //marginals
      for(i=0;i<N;i++){
        f=*E(i);
        for(k=0;k<N;k++) if(k!=i && F(k,i)) tensorMultiply(f,*M(k,i));
        marg(i) = f;
        post(i) = f.P(0);
      }
    //marg.write(cout,"\n  ");
    //MT::wait();
    
    //all messages
      for(i=0;i<N;i++) for(j=0;j<N;j++) if(i!=j && F(i,j)){
        f=*E(i);
        for(k=0;k<N;k++) if(k!=j && k!=i && F(k,i)) tensorMultiply(f,*M(k,i));
        tensorProductMarginal(*M(i,j), *F(i,j), f, ARRAY(V(i)));
      }
    }else{
      //marginals
      for(i=0;i<N;i++){
        f=*E(i);
        for(k=0;k<N;k++) if(i!=k && F(k,i)) tensorMultiply(f,*M1(k,i,i));
        marg(i) = f;
        post(i) = f.P(0);
      }
      //marg.write(cout,"\n  ");
      //MT::wait();
    
      //all messages
      for(i=0;i<N;i++) for(j=0;j<N;j++) if(i!=j && F(i,j)){
      
        //special
        for(l=0;l<N;l++){
          if(i==l) M1(i,j,l)->setOne();
          else{ //collect at i
            f=*E(i);
            for(k=0  ;k<N;k++) if(k!=j && k!=i && F(k,i)){
              if(order==1) tensorMultiply(f,*M1(k,i,l));
              else         tensorMultiply(f,*M2(ARRAY(k,i,l,i)));
            }
            tensorProductMarginal(*M1(i,j,l), *F(i,j), f, ARRAY(V(i)));
          //cout <<"M1("<<i<<","<<j<<","<<l<<","<<l<<")=" <<*M1(i,j,l,l) <<endl;
          }
        }

        if(order==2) for(l=0;l<N;l++) for(ll=l+1;ll<N;ll++){
          if(l==ll) continue;
          if(i==l || i==ll) M2(ARRAY(i,j,l,ll))->setOne();
          else{ //collect at i
            f=*E(i);
            for(k=0  ;k<N;k++) if(k!=j && k!=i && F(k,i)) tensorMultiply(f,*M2(ARRAY(k,i,l,ll)));
            tensorProductMarginal(*M2(ARRAY(i,j,l,ll)), *F(i,j), f, ARRAY(V(i)));
          //cout <<"M2("<<i<<","<<j<<","<<l<<","<<ll<<")=" <<*M2(i,j,l,ll) <<endl;
          }
        }
      }
    }
  }

  double KLD=0.,a,b;
  for(i=0;i<N;i++){
    a=   ex_post(i);  b=   post(i);  KLD += a*(::log(a) - ::log(b));
    a=1.-ex_post(i);  b=1.-post(i);  KLD += a*(::log(a) - ::log(b));
  }

  cout <<"\nex_post = " <<ex_post <<"\npost    = " <<post <<"\nKLD=" <<KLD <<endl;
  
}
