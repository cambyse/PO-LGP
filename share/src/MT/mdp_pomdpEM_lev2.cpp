#include "mdp.h"
#include "mstep.h"
#include "infer.h"

using namespace infer;

#ifndef rescaleRewards
#  define rescaleRewards false
#endif

#undef useStructure
//#define useStructure

double mdp::pomdpEM_lev2(
      const MDP& mdp,
      FSC_lev2& fsc,
      uint T,
      bool structuredEstep,
      bool maxMstep,
      bool adaptP0,
      std::ostream *os){
  
  CHECK(mdp.Px.nd==1 && mdp.Pxax.nd==3 &&
        mdp.Pyxa.nd==3 && mdp.Rax.nd==2 &&
        fsc.Pa0.nd==2 && fsc.P01y0.nd==4,"");
  
  uint
    dx=mdp.Pxax.d0,
    da=mdp.Pxax.d1,
    dy=mdp.Pyxa.d0,
    d0=fsc.P0.N,
    d1=fsc.P1.N;

  //----- rescale rewards if necessary
  arr mdp_Rax = mdp.Rax;
  double Rmin=mdp_Rax.min(),Rmax=mdp_Rax.max();
  if(rescaleRewards || (!maxMstep && Rmin<0.)){
    //if(!rescaleRewards) MT_MSG("can't handle neg rewards in case of exact M-step -- I'm enforcing rescaling of rewards!");
    for(uint i=0;i<mdp_Rax.N;i++) mdp_Rax.elem(i) = (mdp_Rax.elem(i)-Rmin)/(Rmax-Rmin);
  }else{
    Rmin=0.; Rmax=1.;
  }

  MT::timerStart();
  
  //----- define the factor model
  Variable x  (dx ,"state(t)");
  Variable y  (dy ,"observation(t)");
  Variable n1 (d1 ,"node1(t)");
  Variable n0 (d0 ,"node0(t)");
  Variable a  (da ,"action(t)");
  Variable x_ (dx ,"state(t+1)");
  Variable y_ (dy ,"observation(t+1)");
  Variable n1_(d1 ,"node1(t+1)");
  Variable n0_(d0 ,"node0(t+1)");
  //start
  Factor Fx  (ARRAY(&x)        ,mdp.Px);
  Factor F1  (ARRAY(&n1)       ,fsc.P1);
  Factor F0  (ARRAY(&n0)       ,fsc.P0);
  //transition
  Factor Fa0  (ARRAY(&a,&n0)         ,fsc.Pa0);
  Factor Fxax (ARRAY(&x_,&a,&x)       ,mdp.Pxax);
  Factor Fyxa (ARRAY(&y_,&x_,&a)      ,mdp.Pyxa);
  Factor F1y01(ARRAY(&n1_,&y_ ,&n0,&n1),fsc.P1y01);
  Factor F01y0(ARRAY(&n0_,&n1_,&y_,&n0),fsc.P01y0);
  //reward
  Factor FRax(ARRAY(&a,&x)      ,mdp_Rax);
  
  Factor Falpha(ARRAY(&n0 ,&n1 ,&x ));
  Factor Fbeta (ARRAY(&n0_,&n1_,&x_));
  arr PT;
  double PR,ET;
  if(!structuredEstep){
    //----- collapse to unstructured model for generic inference
    //get transition matrix
    Factor F01x01x;
    eliminationAlgorithm(F01x01x, ARRAY(&Fa0, &Fxax, &Fyxa, &F1y01, &F01y0), ARRAY(&n0_,&n1_,&x_,&n0,&n1,&x));
    F01x01x.P.reshape(d0*d1*dx,d0*d1*dx);
    //get reward vector
    Factor FR01x;
    Factor tmp(ARRAY(&n1)); tmp.setOne();
    eliminationAlgorithm(FR01x, ARRAY(&tmp,&Fa0,&FRax), ARRAY(&n0,&n1,&x));
    FR01x.P.reshape(d0*d1*dx);
    //get start vector
    Factor F01x;
    eliminationAlgorithm(F01x, ARRAY(&F0,&F1,&Fx), ARRAY(&n0,&n1,&x));
    F01x.P.reshape(d0*d1*dx);
    
    //----- E-STEP
    arr alpha,beta;
    inferMixLengthUnstructured(alpha,beta,PT,PR,ET,
                              F01x.P,FR01x.P,F01x01x.P,mdp.gamma,T);
    Falpha.setP(alpha);
    Fbeta .setP(beta);
  }else{
    //----- use factor lists for generic inference
    FactorList trans = ARRAY(&Fa0, &Fxax, &Fyxa, &F1y01, &F01y0);
    FactorList newed;
    eliminateVariable(trans,newed,&a);
    //eliminateVariable(trans,newed,y_);
  
    Factor tmp(ARRAY(&n1)); tmp.setOne();
    FactorList rewards = ARRAY(&FRax, &Fa0, &tmp);
    eliminateVariable(rewards,newed,&a);
  
    inferMixLengthStructured(Falpha,Fbeta,PT,PR,ET,
                            ARRAY(&n0 ,&n1, &x ),ARRAY(&n0_,&n1_, &x_),
                            ARRAY(&F0,&F1,&Fx),
                            rewards,
                            trans,
                            mdp.gamma,T);
    
    for(uint i=0;i<newed.N;i++) delete newed(i);
  }
  
  if(os) (*os) <<"E: " <<MT::timerRead(true) <<"sec, M: " <<std::flush;
  
  //----- M-STEP
  //consider the 2nd term (alpha*P_(x'|x)*beta)
  FactorList twotimeslice = ARRAY(&Falpha, &Fa0, &Fxax, &Fyxa, &F1y01, &F01y0, &Fbeta);
  
  Factor X1y01_term2;
  eliminationAlgorithm(X1y01_term2,twotimeslice,ARRAY(&n1_,&y_ ,&n0,&n1));
  Factor X01y0_term2;
  eliminationAlgorithm(X01y0_term2,twotimeslice,ARRAY(&n0_,&n1_,&y_,&n0));
  Factor Xa0_term2;
  eliminationAlgorithm(Xa0_term2,twotimeslice,ARRAY(&a,&n0));
  
  //consider the 1st term (alpha*R_x)
  FactorList immediateR   = ARRAY(&Falpha, &Fa0, &Fxax, &Fyxa, &F1y01, &F01y0, &FRax);

  Factor X1y01_term1;
  eliminationAlgorithm(X1y01_term1 ,immediateR,ARRAY(&n1_,&y_ ,&n0,&n1));
  Factor X01y0_term1;
  eliminationAlgorithm(X01y0_term1 ,immediateR,ARRAY(&n0_,&n1_,&y_,&n0));
  Factor Xa0_term1;
  eliminationAlgorithm(Xa0_term1,immediateR,ARRAY(&a,&n0));
    
  arr X1y01;
  X1y01 = X1y01_term2.P*::exp(X1y01_term2.logP);
    
  arr X01y0;
  X01y0 = X01y0_term2.P*::exp(X01y0_term2.logP);
    
  arr Xa0;
  Xa0 = Xa0_term2.P*::exp(Xa0_term2.logP)
      + Xa0_term1.P*::exp(Xa0_term1.logP);
  
  //do the M-step!
  if(maxMstep){
    noisyMaxMstep_old(fsc.P1y01, X1y01, 1);
    noisyMaxMstep_old(fsc.P01y0, X01y0, 1);
    noisyMaxMstep_old(fsc.Pa0  , Xa0  , 1);
    //if(adaptP0) noisyMaxMstep(fsc.P0   , X0.P   , 1);
  }else{
    standardMstep(fsc.P1y01, X1y01, 1);
    standardMstep(fsc.P01y0, X01y0, 1);
    standardMstep(fsc.Pa0  , Xa0  , 1);
    //if(adaptP0) standardMstep(fsc.P0   , X0.P   , 1);
  }

  tensorCheckCondNormalization(fsc.Pa0  ,1,1e-10);
  tensorCheckCondNormalization(fsc.P1y01,1,1e-10);
  tensorCheckCondNormalization(fsc.P01y0,1,1e-10);
  
  //----- rest is cosmetics
  //report
  if(os) (*os) <<MT::timerRead() <<"sec," <<std::flush;
  if(os) (*os) <<" P(r=1)=" <<PR
        <<", Exp(T)=" <<ET <<"/"<<::log(PR)/::log(mdp.gamma)
        <<", Exp(R)=" <<(PR*(Rmax-Rmin)+Rmin)/(1.-mdp.gamma)
        <<endl;
 
  return (PR*(Rmax-Rmin)+Rmin)/(1.-mdp.gamma);
}
