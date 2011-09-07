#include "mdp.h"
#include "mstep.h"
#include "infer.h"

using namespace infer;

double mdp::pomdpEM_lev1(
  const MDP& mdp,
  FSC_lev1& fsc,
  uint estepHorizon,
  bool estepStructured,
  bool estepIncremental,
  MstepType mstepType,
  double mstepRate,
  double mstepNoise,
  bool adaptP0,
  arr* alpha, arr* beta,
  ostream *os){
  
//   cout  <<"estepHorizon="  <<estepHorizon  <<endl;
//   cout  <<"estepStructured="  <<estepStructured  <<endl;
//   cout  <<"estepIncremental="  <<estepIncremental <<endl;
//   cout  <<"mstepType="  <<mstepType <<endl;
//   cout  <<"mstepRate="  <<mstepRate <<endl;
//   cout  <<"mstepNoise="  <<mstepNoise <<endl;
//   cout  <<"adaptP0="  <<adaptP0 <<endl;

  MT::timerStart();
  
  CHECK(mdp.Px.nd==1 && mdp.Pxax.nd==3 &&
        mdp.Pyxa.nd==3 && mdp.Rax.nd==2 &&
        fsc.Pa0.nd==2 && fsc.P0y0.nd==3, "");
        
  uint
  dx=mdp.Pxax.d0,
     da=mdp.Pxax.d1,
        dy=mdp.Pyxa.d0,
           d0=fsc.P0y0.d0;
           
  //----- rescale rewards if necessary
  arr mdp_Rax = mdp.Rax;
#if rescaleRewards
  double Rmin=mdp_Rax.min(), Rmax=mdp_Rax.max();
  if(rescaleRewards || (mstepType!=MstepNoisyMax && Rmin<0.)){
    //if(!rescaleRewards) MT_MSG("can't handle neg rewards in case of exact M-step -- I'm enforcing rescaling of rewards!");
    for(uint i=0; i<mdp_Rax.N; i++) mdp_Rax.elem(i) = (mdp_Rax.elem(i)-Rmin)/(Rmax-Rmin);
  }else{
    Rmin=0.; Rmax=1.;
  }
#endif
  
  //----- define the factor model
  Variable x(dx , "state(t)");
  Variable y(dy , "observation(t)");
  Variable n0(d0 , "node0(t)");
  Variable a(da , "action(t)");
  Variable x_(dx , "state(t+1)");
  Variable y_(dy , "observation(t+1)");
  Variable n0_(d0 , "node0(t+1)");
  //start
  Factor Fx(ARRAY(&x)        , mdp.Px);
  Factor Fy(ARRAY(&y));      Fy.setUniform();
  Factor F0(ARRAY(&n0)       , fsc.P0);
  //transition
  Factor Fa0(ARRAY(&a, &n0)     , fsc.Pa0);
  Factor Fxax(ARRAY(&x_, &a, &x)   , mdp.Pxax);
  Factor Fyxa(ARRAY(&y_, &x_, &a)  , mdp.Pyxa);
  Factor F0y0(ARRAY(&n0_, &y_, &n0), fsc.P0y0);
  //reward
  Factor FRax(ARRAY(&a, &x)      , mdp_Rax);
  
  VariableList leftVars=ARRAY(&n0 , &x);
  VariableList rightVars=ARRAY(&n0_, &x_);
  VariableList tail_headVars=cat(rightVars, leftVars);
  
  //FactorList allTransitions=ARRAY(&Fa0, &Fxax, &Fyxa, &F0y0);
  FactorList allTransitions = ARRAY(&F0y0, &Fyxa, &Fxax, &Fa0);
  FactorList allRewards = ARRAY(&FRax, &Fa0);
  FactorList allInits = ARRAY(&F0, &Fx, &Fy);
  
  Factor Falpha(leftVars);
  Factor Fbeta(rightVars);
  arr PT;
  double PR, ET;
  if(!estepStructured){
    //----- collapse to unstructured model for generic inference
    uint dz = d0*dx;
    //get transition matrix
    Factor Fzz;
    eliminationAlgorithm(Fzz, allTransitions, tail_headVars);
    Fzz.P.reshape(dz, dz);
    //get reward vector
    Factor FRz;
    eliminationAlgorithm(FRz, allRewards, leftVars);
    FRz.P.reshape(dz);
    //get start vector
    Factor Fz;
    eliminationAlgorithm(Fz, allInits, leftVars);
    Fz.P.reshape(dz);
    
    //MT::save(Fz, "z.gFz");
    //MT::save(FRz, "z.gFRz");
    //MT::save(Fzz, "z.gFzz");
    
    //----- E-STEP
    arr _alpha, _beta;
    if(estepIncremental){
      _alpha.referTo(*alpha);  _alpha.reshape(dz);
      _beta .referTo(*beta);   _beta .reshape(dz);
    }
    
    inferMixLengthUnstructured(_alpha, _beta, PT, PR, ET,
                               Fz.P, FRz.P, Fzz.P, mdp.gamma, estepHorizon,
                               estepIncremental);
    Falpha.setP(_alpha);
    Fbeta .setP(_beta);
    
    if(!estepIncremental){
      if(alpha)(*alpha)=Falpha.P;
      if(beta)(*beta) =Fbeta.P;
    }
  }else{
    //----- use factor lists for generic inference
    FactorList temporary;
    //eliminateVariable(allTransitions, temporary, a);
    //eliminateVariable(allTransitions, temporary, y_);
    //eliminateVariable(allRewards, temporary, a);
    
    if(estepIncremental){
      Falpha.setP(*alpha);
      Fbeta .setP(*beta);
    }
    
    inferMixLengthStructured(Falpha, Fbeta, PT, PR, ET,
                             leftVars, rightVars,
                             allInits,
                             allRewards,
                             allTransitions,
                             mdp.gamma, estepHorizon,
                             estepIncremental);
                             
    if(alpha)(*alpha)=Falpha.P;
    if(beta)(*beta) =Fbeta.P;
    
    for(uint i=0; i<temporary.N; i++) delete temporary(i);
  }
  
  if(os)(*os)  <<"E: "  <<MT::timerRead(true)  <<"sec, M: "  <<std::flush;
  
  //----- M-STEP
  //term2: derived from the full two-time-slice model (beta*P_(x'|x)*alpha)
  FactorList twotimeslice = ARRAY(&Falpha, &Fa0, &Fxax, &Fyxa, &F0y0, &Fbeta);
  
  Factor X0y0_term2;
  eliminationAlgorithm(X0y0_term2, twotimeslice, ARRAY(&n0_, &y_, &n0));
  Factor Xa0_term2;
  eliminationAlgorithm(Xa0_term2, twotimeslice, ARRAY(&a, &n0));
  
  //consider the 1st term (alpha*R_x)
  FactorList immediateR   = ARRAY(&Falpha, &Fa0, &Fxax, &Fyxa, &F0y0, &FRax);
  
  Factor X0y0_term1;
  eliminationAlgorithm(X0y0_term1 , immediateR, ARRAY(&n0_, &y_, &n0));
  Factor Xa0_term1;
  eliminationAlgorithm(Xa0_term1, immediateR, ARRAY(&a, &n0));
  
  arr X0y0;
#if 0 //this parameter does not depend on immediate reward (only term2 is relevant)
  X0y0 = X0y0_term2.P*::exp(X0y0_term2.logP)
         + X0y0_term1.P*::exp(X0y0_term1.logP);
#else
  X0y0 = X0y0_term2.P*::exp(X0y0_term2.logP);
#endif
         
  arr Xa0;
  Xa0 = Xa0_term2.P*::exp(Xa0_term2.logP)
        + Xa0_term1.P*::exp(Xa0_term1.logP);
        
  //do the M-step!
  switch(mstepType){
    case MstepNoisyMax:
      noisyMaxMstep(fsc.P0y0, X0y0, 1, mstepRate, mstepNoise);
      noisyMaxMstep(fsc.Pa0 , Xa0 , 1, mstepRate, mstepNoise);
      //if(adaptP0) noisyMaxMstep(fsc.P0   , X0.P   , 1);
      tensorCheckCondNormalization(fsc.Pa0 , 1);
      tensorCheckCondNormalization(fsc.P0y0, 1);
      break;
    case MstepExact:
      standardMstep(fsc.P0y0, X0y0, 1, mstepNoise);
      standardMstep(fsc.Pa0 , Xa0 , 1, mstepNoise);
      //if(adaptP0) standardMstep(fsc.P0   , X0.P   , 1);
      tensorCheckCondNormalization(fsc.Pa0, 1);
      tensorCheckCondNormalization(fsc.P0y0, 1);
      break;
    case Mstep11Rule:
      mstep_11rule(fsc.P0y0, X0y0, 1, mstepRate, mstepNoise);
      mstep_11rule(fsc.Pa0 , Xa0 , 1, mstepRate, mstepNoise);
      //if(adaptP0) mstep_11rule(fsc.P0   , X0   , 1, 1.1);
      tensorCheckCondNormalization(fsc.P0 , 1);
      tensorCheckCondNormalization(fsc.Pa0 , 1);
      tensorCheckCondNormalization(fsc.P0y0, 1);
      break;
    case MstepNone:
      break;
    case MstepCopyExpectations:
      fsc.P0y0= X0y0;
      fsc.Pa0 = Xa0 ;
      break;
    default:
      NIY;
  }
  
  if(adaptP0) NIY;
  
  //in case we rescaled, reset
  double expR;
#if rescaleRewards
  expR=(PR*(Rmax-Rmin)+Rmin)/(1.-mdp.gamma);
#else
  expR=PR/(1.-mdp.gamma);
#endif
  
  //----- rest is cosmetics
  //report
  if(os)(*os)  <<MT::timerRead()  <<"sec, "  <<std::flush;
  if(os)
    (*os)  <<" P(r=1)="  <<PR
   <<", Exp(T)="  <<ET  <<"/" <<::log(PR)/::log(mdp.gamma)
   <<", Exp(R)="  <<expR
   <<endl;
    
  return expR;
}
