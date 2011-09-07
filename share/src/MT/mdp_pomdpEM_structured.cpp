#include "mdp.h"
#include "mstep.h"
#include "infer.h"

using namespace infer;

double mdp::pomdpEM_structured(
  const MDP_structured& mdp,
  FSC_structured& fsc,
  uint estepHorizon,
  bool estepStructured,
  bool estepIncremental,
  MstepType mstepType,
  double mstepRate,
  double mstepNoise,
  bool adaptP0,
  arr* alpha, arr* beta,
  ostream *os){
  
  checkConsistent(mdp.facs);
  checkConsistent(fsc.facs);
  
  MT::timerStart();
  
  uint i;
  //----- rescale rewards if necessary
#if rescaleRewards
  Factor *Rax=listFindName(mdp.facs, "Rax");
  arr mdp_Rax_org;
  Rax->getP(mdp_Rax_org);
  arr mdp_Rax = mdp_Rax_org;
  double Rmin=mdp_Rax.min(), Rmax=mdp_Rax.max();
  if(rescaleRewards || (mstepType!=MstepNoisyMax && Rmin<0.)){
    //if(!rescaleRewards) MT_MSG("can't handle neg rewards in case of exact M-step -- I'm enforcing rescaling of rewards!");
    for(i=0; i<mdp_Rax.N; i++) mdp_Rax.elem(i) = (mdp_Rax.elem(i)-Rmin)/(Rmax-Rmin);
  }else{
    Rmin=0.; Rmax=1.;
  }
  Rax->setP(mdp_Rax);
#endif
  
  VariableList leftVars=cat(fsc.leftVars, mdp.leftVars);
  VariableList rightVars=cat(fsc.rightVars, mdp.rightVars);
  VariableList tail_headVars=cat(rightVars, leftVars);
  
  FactorList allTransitions = cat(fsc.transFacs, mdp.obsFacs, mdp.transFacs);
  FactorList allRewards = cat(mdp.rewardFacs, allTransitions);
  FactorList allInits = cat(fsc.initFacs, mdp.initFacs);
  
  Factor Falpha(leftVars);
  Factor Fbeta(rightVars);
  arr PT;
  double PR, ET;
  if(!estepStructured){
    //----- collapse to unstructured model for generic inference
    uint dz = 1;  for(i=0; i<leftVars.N; i++) dz *= leftVars(i)->dim;
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
    
    //MT::save(Fz, "z.sFz");
    //MT::save(FRz, "z.sFRz");
    //MT::save(Fzz, "z.sFzz");
    
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
  FactorList twotimeslice = cat(ARRAY(&Fbeta), fsc.transFacs, mdp.obsFacs, mdp.transFacs, ARRAY(&Falpha));
  
  //term1: derived from the immediate reward model
  FactorList immediateR = cat(mdp.rewardFacs, fsc.transFacs, mdp.obsFacs, mdp.transFacs, ARRAY(&Falpha));
  
  //loop through all transition factors of the controller
  for(i=0; i<fsc.transFacs.N; i++){
    //term2: terms from the two-time-slice model
    Factor X_term2;
    eliminationAlgorithm(X_term2, twotimeslice, fsc.transFacs(i)->variables);
    
    //term1: terms from immediate reward
    Factor X_term1;
    eliminationAlgorithm(X_term1, immediateR  , fsc.transFacs(i)->variables);
    
    //get the expectations by adding both terms
    arr X;
    X = X_term2.P*::exp(X_term2.logP) + X_term1.P*::exp(X_term1.logP);
    
    //do the Mstep
    fsc.transFacs(i)->logP=0.;
    switch(mstepType){
      case MstepNoisyMax:
        noisyMaxMstep(fsc.transFacs(i)->P, X, 1, mstepRate, mstepNoise);
        tensorCheckCondNormalization(fsc.transFacs(i)->P, 1);
        break;
      case MstepExact:
        standardMstep(fsc.transFacs(i)->P, X, 1, mstepNoise);
        tensorCheckCondNormalization(fsc.transFacs(i)->P, 1);
        break;
      case MstepNone:
        break;
      case MstepCopyExpectations:
        fsc.transFacs(i)->P = X;
        break;
      default:
        NIY;
    }
  }
  
  if(adaptP0) NIY;
  
  //in case we rescaled, reset
  double expR;
#if rescaleRewards
  Rax->setP(mdp_Rax_org);
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
