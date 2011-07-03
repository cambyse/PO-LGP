/*  Copyright 2009 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/> */

/** \file aico.h
    \brief Approximate Inference Control */

#ifndef MT_aico_h
#define MT_aico_h

#include "soc.h"

/** \brief Apprioximate Inference Control */
struct AICO_clean{
  //parameters
  soc::SocSystemAbstraction *sys;
  double convergenceRate,repeatThreshold,recomputeTaskThreshold,maxStep;
  double damping,tolerance;
  uint max_iterations,seed;
  uint display;
  bool useBwdMsg;
  arr bwdMsg_v,bwdMsg_Vinv;

  enum StepMethod{ smClean=0, smDynamic, smKinematic, smGaussNewton, smIlqg  };
  int method;

  MT::String filename;
  std::ostream *os;

  //TODO: rename q->x (not position trajectory!)
  //messages
  arr s,Sinv,v,Vinv,r,R,rhat;     //!< fwd, bwd, and task messages
  arr s_old,Sinv_old,v_old,Vinv_old,r_old,R_old,rhat_old;     //!< fwd, bwd, and task messages
  MT::Array<arr> phiBar,JBar;     //!< all task cost terms
  arr Psi;                        //!< all transition cost terms
  arr b,Binv;                     //!< beliefs
  arr q,qhat;                     //!< trajectory (MAP), and point of linearization
  arr b_old,Binv_old,q_old,qhat_old;
  arr dampingReference;
  double cost,cost_old;                      //!< cost of MAP trajectory
  double b_step;
  
  // INTERNAL
  bool useFwdMessageAsQhat;
  arr A,tA,Ainv,invtA,a,B,tB,Winv,Hinv,Q; //!< processes...
  uint sweep;                     //!< #sweeps so far
  uint scale;                     //!< scale of this AICO in a multi-scale approach

  AICO_clean(){ sweep=0; scale=0; maxStep=.1; method=smClean; }
  AICO_clean(soc::SocSystemAbstraction &sys){ sweep=0; scale=0; maxStep=.1; init(sys); }

  //-- high level access
  void init(soc::SocSystemAbstraction &sys);
  void iterate_to_convergence(const arr* q_initialization=NULL);

  void init(soc::SocSystemAbstraction& _sys,
            double _convergenceRate,double _repeatThreshold,
	    double _recomputeTaskThreshold,
            uint _display, uint _scale);
  void init_messages();
  void shift_solution(int offset);

  double step(){
    switch(method){
      case smClean:       return stepSweeps(); //stepClean();
      case smDynamic:     return stepDynamic();
      case smKinematic:   return stepKinematic();
      case smIlqg:        return stepIlqg();
      case smGaussNewton: return stepGaussNewton();
    }
    HALT("");
    return 0.;
  }
  double stepClean();
  double stepDynamic();
  double stepKinematic();
  double stepIlqg(){NIY;};
  double stepGaussNewton();
  double stepMinSum();
  double stepSweeps();

  //internal helpers
  void initMessagesWithReferenceQ(const arr& qref); //use to initialize damping reference!!!
  void initMessagesFromScaleParent(AICO_clean *parent);
  void updateFwdMessage(uint t);
  void updateBwdMessage(uint t);

  void updateTimeStep(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations, double tolerance, bool forceRelocation);
  void updateTimeStepGaussNewton(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations, double tolerance);
  double evaluateTimeStep(uint t,bool includeDamping);
  double evaluateTrajectory(const arr& x);

  void displayCurrentSolution();
  void rememberOldState();
  void perhapsUndoStep();

};

void AICO_multiScaleSolver(soc::SocSystemAbstraction& sys,
                           arr& q,
                           double tolerance,
                           double convergenceRate,double repeatThreshold, double recomputeTaskThreshold,
                           uint display,
                           uint scalePowers);

#if 0

inline void getController(arr& G,arr& g,const AICO_clean& aico){
  //we can only compute a controller for time steps 0 to T-1 (based on V_{t+1})
  uint T=aico.s.d0-1;
  uint n=aico.s.d1;
  if(!aico.sys->dynamic){
    G.resize(T,n,n);
    g.resize(T,n);
  }else{
    G.resize(T,n/2,n);
    g.resize(T,n/2);
  }
  arr H;
  for(uint t=0;t<T;t++){
    arr Vstar,barv,VstarH;
    aico.sys->getH(H,t);
    if(!aico.sys->dynamic){
      //controller model u_mean = G*x+g
      Vstar = aico.Vinv[t+1] + aico.R[t+1];
      lapack_Ainv_b_sym(barv, Vstar, aico.Vinv[t+1]*aico.v[t+1] + aico.r[t+1]);
      inverse_SymPosDef(VstarH,Vstar + H);
      G[t] = - VstarH * Vstar; // * aico.A[t];
      g[t] = VstarH * Vstar * (barv); // - aico.a[t]);
    }else{
      Vstar = aico.Vinv[t+1] + aico.R[t+1];
      lapack_Ainv_b_sym(barv, Vstar, aico.Vinv[t+1]*aico.v[t+1] + aico.r[t+1]);
      inverse_SymPosDef(VstarH,aico.tB[t]*Vstar*aico.B[t] + H);
      G[t] = - VstarH * aico.tB[t] * Vstar * aico.A[t];
      g[t] = VstarH * aico.tB[t] * Vstar * (barv - aico.a[t]);
    }
  }
}

inline void forwardSimulateTrajectory(arr& q,const arr& G,const arr& g,soc::SocSystemAbstraction& sys,const soc::AICO_clean& aico){
  uint t,T=sys.nTime(),n=sys.qDim();
  if(!aico.sys->dynamic){
    q.resize(T+1,n);
    sys.getq0(q[0]());
    for(t=0;t<T;t++) q[t+1]() = q[t] + (G[t]*q[t] + g[t]); //A=1, B=1
  }else{
    q.resize(T+1,2*n);
    sys.getqv0(q[0]());
    for(t=0;t<T;t++) q[t+1]() = aico.A[t]*q[t] + aico.B[t]*(G[t]*q[t] + g[t]) + aico.a[t];
    arr q_sub;
    getPositionTrajectory(q_sub,q);
    q=q_sub;
  }
}

inline void getControlledTrajectory(arr& q,const soc::AICO_clean& aico){
  arr G,g;
  getController(G,g,aico);
  forwardSimulateTrajectory(q,G,g,*aico.sys,aico);
}
#endif

//===========================================================================
//
// implementations
//

#ifdef  MT_IMPLEMENTATION
#  include "aico.cpp"
#endif

#endif
