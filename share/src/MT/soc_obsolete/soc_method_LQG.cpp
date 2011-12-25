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

#include "soc.h"
#include "util.h"

void soc::LQG::init(SocSystemAbstraction& _sys,
                     double _convergenceRate, uint _display,
                     uint _scale)
{
  sys = &_sys;  //sys.clone(_sys);
  convergenceRate=_convergenceRate;
  display=_display;
  scale=_scale;
  cost=-1;
}


void soc::LQG::shiftSolution(int offset){
  uint n=sys->qDim();
  arr q0;
#if 0 //trust that the system knows 10!
  if(!sys->dynamic){ sys->getq0(q0); }else{ sys->getqv0(q0);  n*=2; }
#else //take q0 to be the one specified by hatq[offset]!
  q0=q_phase[offset];
  if(!sys->dynamic){ sys->setq(q0); }else{ sys->setx(q0);  n*=2; }
  sys->setq0AsCurrent();
#endif
  vbar.shift(offset*n, false);  Vbar.shift(offset*n*n, false);
  r.shift(offset*n, false);  R.shift(offset*n*n, false);  r[0]=0.;      R[0]=0.;
  q_phase.shift(offset*n, false);  q_phase[0]=q0;
}


/*! \brief iterated LQG (linear quadratic Gaussian) applied to the
    kinematic trajectory optimization case. This means that we assume
    A=B=1 (simple additive control) with control cost H=W
    (corresponding directly to the q-space metric). The quadratic cost
    terms are computed from the tast constraints */
double soc::LQG::stepKinematic(){
  uint t, T=sys->nTime(), n=sys->qDim();

  //arr Vbar(T+1, n, n), vbar(T+1, n);
  //arr R(T+1, n, n), r(T+1, n);
  arr HVinv(T+1, n, n), VHVinv;
  //we assume A=B=\id for now
  arr H;

  //remember the old trajectory
  arr q_old(q);

  if(q.nd==2 && q.d0==(T>>1)+1){ //upscale...
    q.resize(T+1, q.d1);
    for(t=0;t<=T;t+=2){
      q[t] = q_old[t>>1];  if(t<T){ q[t+1] = (double).5*(q_old[t>>1] + q_old[(t>>1)+1]); }
    }
    q_old=q;
  }
     
  //trajectory needs to be initialized!
  CHECK(q.nd==2 && q.d0==T+1 && q.d1==n, "please initialize trajectory!");
  //possible initialization routines:
  //  straightTaskTrajectory(soci, q, 0);
  //  bayesianIKTrajectory(soci, q);
  //  sys->passiveDynamicTrajectory(q);


  if(!sweep){
    if(sys->os){
#ifdef NIKOLAY
      *sys->os <<std::setw(3) <<-1 <<" " <<MT::timerRead(false);
      sys->totalCost(NULL, q, false);
#else
      *sys->os <<"LQG " <<std::setw(3) <<-1 <<"  time " <<MT::timerRead(false);
      sys->analyzeTrajectory(q, display>0);
      //sys->computeTotalCost(q);
#endif
    }
    if(sys->gl){
      sys->displayTrajectory(q, NULL, display, STRING("LQG optimization -- iteration " <<-1));
    }
  }

  Vbar.resize(T+1, n, n);  vbar.resize(T+1, n);
  R.resize(T+1, n, n);  r.resize(T+1, n);

  //linearize around current trajectory
  for(t=0;t<=T;t++){
    countSetq++;
    sys->setq(q[t]);
    //sys->getQuadraticTaskCost(R[t](), r[t](), q[t], t);
    sys->getTaskCosts(R[t](), r[t](), q[t], t);
    r[t]() *= -2.;
  }

  //bwd Ricatti equations
  Vbar[T]() = R[T];
  vbar[T]() = r[T];
  for(t=T;t--;){
    sys->getW(H, t);
    inverse_SymPosDef(HVinv[t+1](), H+Vbar[t+1]);
    VHVinv = Vbar[t+1]*HVinv[t+1];
    Vbar[t]() = R[t] + Vbar[t+1] - VHVinv*Vbar[t+1];
    vbar[t]() = r[t] + vbar[t+1] - VHVinv*vbar[t+1];
  }

  //fwd with optimal control
  sys->getq0(q[0]());
  for(t=1;t<=T;t++){
    if(convergenceRate==1.){
      q[t]() = q[t-1] - HVinv[t]*((double).5*vbar[t] + Vbar[t]*q[t-1]);
    }else{
      q[t]() = ((double)1.-convergenceRate)*q[t]
	+ convergenceRate*(q[t-1] - HVinv[t]*((double).5*vbar[t] + Vbar[t]*q[t-1]));
    }
  }

  sweep++;
  
  double diff = -1.;
  if(q_old.N==q.N) diff=maxDiff(q_old, q);

  //display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"LQGk(" <<sys->scalePower <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" diff " <<diff;
   cost= sys->analyzeTrajectory(q, display>0);
    //sys->computeTotalCost(q);
  }
  if(display){
    sys->displayTrajectory(q, NULL, display, STRING("LQG optimization -- iteration "));
  }
  MT::timerResume();

  return diff;
}

//===========================================================================

/*! \brief iterated LQG (linear quadratic Gaussian) for the general
    Stochastic Optimal Control case */
double soc::LQG::stepGeneral(){
  CHECK(sys->dynamic, "assumed dynamic SOC abstraction");
  uint t, T=sys->nTime(), n=sys->qDim(), n2;

  n2=2*n;
  arr A(T+1, n2, n2), a(T+1, n2), B(T+1, n2, n);
  arr At, Bt, VA, HVinv, BIG, u; //helpers
  arr H;

  double tau=sys->getTau();

  //remember the old trajectory
  arr q_old(q);
  double cost_old=cost;
  
  if(q.nd==2 && q.d0==(T>>1)+1){ //upscale...
    q.resize(T+1, q.d1);
    for(t=0;t<=T;t+=2){
      q[t] = q_old[t>>1];  if(t<T){ q[t+1] = (double).5*(q_old[t>>1] + q_old[(t>>1)+1]); }
    }
    q_old=q;
    sweep=0;
  }
     
  //trajectory needs to be initialized!
  CHECK(q.nd==2 && q.d0==T+1 && q.d1==n, "please initialize trajectory!");

  //get velocity profile
  getPhaseTrajectory(q_phase, q, tau);

  Vbar.resize(T+1, n2, n2);  vbar.resize(T+1, n2);
  R.resize(T+1, n2, n2);  r.resize(T+1, n2);
  
  //linearize around current trajectory
  for(t=0;t<=T;t++){
    countSetq++;
    sys->setx(q_phase[t]);
    sys->getCosts  (R[t](), r[t](), q[t], t);
    sys->getProcess(A[t](), a[t](), B[t](), t);
    //cout <<"t=" <<t <<" A=" <<A[t] <<" a=" <<a[t] <<" B=" <<B[t] <<endl;
    //cout <<"t=" <<t <<" R=" <<R[t] <<" r=" <<r[t] <<endl;
  }

  //bwd Ricatti equations
  Vbar[T]() = R[T];
  vbar[T]() = r[T];
  for(t=T;t--;){
    sys->getH(H, t);
    transpose(At, A[t]);
    transpose(Bt, B[t]);
    innerProduct(VA, Vbar[t+1], A[t]);
    inverse_SymPosDef(HVinv, H+Bt*Vbar[t+1]*B[t]);
    BIG = (~VA)*B[t]*HVinv*Bt;
    Vbar[t]() = R[t] + (At-BIG)*VA;
    vbar[t]() = r[t] + (At-BIG)*(vbar[t+1]-Vbar[t+1]*a[t]);
    //if(t==0) cout <<"\nt=" <<t <<"\nV=" <<Vbar[t] <<"\nv=" <<vbar[t] <<endl;
  }

  //fwd with optimal control
  //we assume that q_phase[0] is fine and fixed!    (from getPhaseTrajectory)
  double ctrlC=0;
  for(t=0;t<T;t++){
    countSetq++;
    sys->setx(q_phase[t]);
    sys->getH(H, t);
    sys->getProcess(A[t](), a[t](), B[t](), t);
    //if(t>T-10) cout <<"t=" <<t <<"\nqhat=" <<q_phase[t] <<endl;

    transpose(Bt, B[t]);
    inverse_SymPosDef(HVinv, H+Bt*Vbar[t+1]*B[t]);
    u = - HVinv*Bt*(Vbar[t+1]*(A[t]*q_phase[t]+a[t]) - vbar[t+1]);
    ctrlC += scalarProduct(H, u, u);
    //cout <<"ilqg time " <<t <<" u=" <<u <<" q=" <<q_phase[t] <<endl;
    //if(t>T-10) cout <<"t=" <<t+1 <<"\nb=" <<A[t]*q_phase[t] + a[t] + B[t]*u <<endl;
      
    if(convergenceRate==1.){
      q_phase[t+1]() = A[t]*q_phase[t] + a[t] + B[t]*u;
    }else{
      q_phase.reshape(T+1, 2, n);
      q_phase.subDim(t+1, 1)
        = ((double)1.-convergenceRate)*q_phase.subDim(t+1, 1)
        + convergenceRate*(q_phase.subDim(t, 1) + a[t].sub(n, -1) + B[t].sub(n, -1, 0, -1)*u);
      q_phase.subDim(t+1, 0) = q_phase.subDim(t, 0) + tau*q_phase.subDim(t+1, 1);
      q_phase.reshape(T+1, 2*n);
    }
  }

  //compute messages in other form
  /*Vinv.resizeAs(Vbar);  v.resizeAs(vbar);
  if(sweep>2){
    Vinv[T].setDiag(1e-0);
    arr Eps; Eps.setDiag(1e-10, Vinv.d1);
    v[T]() = q_phase[T];
    for(t=0;t<T;t++){
      Vinv[t]() = Vbar[t]-R[t];
      lapack_Ainv_b_sym(v[t](), Vinv[t]+Eps, vbar[t]-r[t]);
    }
  }else{
    Vinv.setZero(); v.setZero();
  }*/
  
  sweep++;

  getPositionTrajectory(q, q_phase);
  double diff=maxDiff(q_old, q);
  cost = sys->analyzeTrajectory(q, display>0);
  if(sweep>1){
    CHECK(cost!=-1, "");
    if(cost>cost_old){
      convergenceRate *= .8;
      cout <<" LQG REJECT" <<endl;
      q = q_old;
      cost = cost_old;
    }else{
      convergenceRate = pow(convergenceRate, .7);
      cout <<" LQG ACCEPT" <<endl;
    }
  }    

  //display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"LQGd(" <<sys->scalePower <<", " <<convergenceRate <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" diff " <<diff;
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("LQG optimization -- iteration "));
  }
  MT::timerResume();

  return diff;
}
