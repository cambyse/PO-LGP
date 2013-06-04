/* OLD STEP METHODS OR AICO_CLEAN

work well, but have been simplified to the cleaner step methods with different sweep modes

NOTE: the stepKinematic includes still the code for the truncated Gaussians

*/

/* VERSIONS
  check out r3261 so see old code parts -- I cleaned them away
*/

//#include <Core/util.h>
//#include "truncatedGaussian.h"
#include "MinSumGaussNewton.h"


/// Approximate Inference Control (AICO_clean) in the kinematic case
double AICO_clean::stepKinematic(){
  CHECK(!sys->dynamic, "assumed dynamic SOC abstraction");
  uint n=sys->qDim();
  uint T=sys->get_T();
  uint t, t0=0;
  int dt;

  //variables for the dynamics
  arr q0;
  sys->getq0(q0);
  Winv.resize(T+1, n, n);
  sys->setq(q0, 0);
  if(!sys->dynamic) sys->getWinv(Winv[0](), 0);
  
  //temporary variables
  arr Vt, St, barS, barV, K, K2;

  //initializations (initial q or multiscale)
  bool useFwdMessageAsInitialQhat=true;
  if(!sweep){
    //-- in case an explicit initialization is given
    if(q.N){
      CHECK(q.nd==2 && q.d0==T+1 && q.d1==n, "initial trajectory was wrong dimensionality");
      useFwdMessageAsInitialQhat=false;
      qhat=q;
      b=qhat;
      v=qhat;  for(uint t=0;t<=T;t++){ Vinv[t].setDiag(1e6);  }
    }

    //-- initialize messages from lower scale
    if(false){ //parent){
      NIY;
      /*
      AICO_clean *A = parent;
      for(t=0;t<=T;t+=2){
        //s[t] = A->s[t>>1]; Sinv[t] = A->Sinv[t>>1];   if(t<T){ s[t+1]=s[t]; Sinv[t+1]=Sinv[t]; }  //don't need to copy fwd messages
        v[t] = A->v[t>>1]; Vinv[t] = A->Vinv[t>>1];  if(t<T){ v[t+1]=A->v[(t>>1)+1]; Vinv[t+1]=A->Vinv[(t>>1)+1]; }
//      if(t<T){ v[t+1]=.5*(A->v[t>>1] + A->v[(t>>1)+1]); Vinv[t+1]=.5*(A->Vinv[t>>1] + A->Vinv[(t>>1)+1]); }
        qhat[t] = A->qhat[t>>1];  if(t<T){ qhat[t+1] = (double).5*(A->qhat[t>>1] + A->qhat[(t>>1)+1]); }
        //b   [t] = A->b   [t>>1];   if(t<T) b   [t+1] = .5*(A->b   [t>>1] + A->b   [(t>>1)+1]);    //don't need to copy the belief
      }
      useFwdMessageAsInitialQhat=false;
      q=qhat;
      b=qhat;
      */
    }

    //-- perhaps display the initialization
    if(q.N){
      if(sys->os){//type initial value
        *sys->os <<"AICO_k(" <<scale <<") " <<std::setw(3) <<-1 <<" time " <<MT::timerRead(false) <<" diff -1";
        sys->analyzeTrajectory(b, display>0);
      }
      if(sys->gl){
        sys->displayTrajectory(q, NULL, display, STRING("AICO_kinematic - iteration - INITIALIZATION"));
      }
    }
  }else{
    //in case q0 changed, reassign the respective messages:
    s[0]=q0;
    b[0]=q0;
    qhat[0]=q0;
  }

  //remember the old trajectory and old qhat
  arr q_old(q), qhat_of_R(qhat);
  
  uint repeatCount=0;

  for(dt=1;dt>=-1;dt-=2){ //fwd & bwd
    if(dt==1)  t0=1;
    if(dt==-1) t0=T;
    for(t=t0;t<=T && t>0;t+=dt){
      //compute (s, S)
      if(dt==1 && !repeatCount){ //only on fwd pass and non-repeats
        countMsg++;
#ifndef TightMode
        inverse_SymPosDef(barS, Sinv[t-1] + R[t-1]);
        s[t] = barS * (Sinv[t-1]*s[t-1] + r[t-1]);
        St = Winv[t-1] + barS;
        inverse_SymPosDef(Sinv[t](), St);
        // I deleted the canonical version (see r3261!; matrix multiplications are slow...
#else
        s[t] = qhat[t-1];
        St = Winv[t-1];
        inverse_SymPosDef(Sinv[t](), St);
#endif
        //cout <<"s\n" <<s[t] <<endl <<Sinv[t] <<endl;
      }

      //compute (v, V)
      if(dt==-1 && !repeatCount){ //only on bwd pass and non-repeats
        countMsg++;
        if(t<T){
          inverse_SymPosDef(barV, Vinv[t+1] + R[t+1]);   //eq (*)
          v[t] = barV * (Vinv[t+1]*v[t+1] + r[t+1]);
          Vt = Winv[t] + barV;
          inverse_SymPosDef(Vinv[t](), Vt);
          // I deleted the canonical version (see r3261!; matrix multiplications are slow...
        }
        if(t==T){ //last time slice
          v[t] = qhat[t]; //alternatives: qhat or b
#ifndef TightMode
          Vinv[t].setDiag(1e-0); //regularization, makes eq (*) above robust
#else
          Vinv[t].setDiag(1e-1); //regularization, makes eq (*) above robust
#endif
        }
      //  cout <<"v\n" <<v[t] <<endl <<Vinv[t] <<endl;
      }
        
      //first sweep and no initialization: set qhat equal to fwd message
      if(!sweep && useFwdMessageAsInitialQhat) qhat[t]()=s[t];

      //compute (r, R) and process
      if(!sweep || maxDiff(qhat[t], qhat_of_R[t])>=recomputeTaskThreshold){ //recompute only when significant change of state
        countSetq++;
        sys->setq(qhat[t], t);
        arr Rt, rt;
        sys->getTaskCosts(Rt, rt, qhat[t], t);
#if 1
        R[t] = Rt; r[t] = rt;
#else
        if(!sweep){
          R[t] = Rt; r[t] = rt;
        }else{
          double eps=5./sweep;
          if(eps>1.) eps=1.;
          R[t] = (1.-eps)*R[t] + eps*Rt;
          r[t] = (1.-eps)*r[t] + eps*rt;
        }
#endif
        
        qhat_of_R[t]() = qhat[t];
        //cout <<"r\n" <<r[t] <<endl <<R[t] <<endl;
      }
      //else cout <<"skip." <<flush;

      //compute system matrices
      if(!sys->dynamic) sys->getWinv(Winv[t](), t);

      //compute (b, B);
      Binv[t] = Sinv[t] + Vinv[t] + R[t];
      lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
      //cout <<"b\n" <<b[t] <<endl <<B[t] <<endl;

#if USE_TRUNCATION //PRELIMINARY - hard constraints handled with truncating Gaussians
      //sys->displayState(b[t], &Binv[t], STRING("AICO kinematic (online) t=" <<t));
      //account for constraints:
      arr cdir, coff;
      sys->setq(b[t], t);
      //sys->gl->watch(STRING("time " <<t));
      sys->getConstraints(cdir, coff, t <<scale, b[t]);
      if(cdir.d0){
        //cout <<"t=" <<t <<' ';
        arr __b, __B, __Binv;
        inverse_SymPosDef(__B, Binv[t]);
        __b=b[t];
        //plotClear();  plotCovariance(__b, __B);
        for(uint i=0;i<cdir.d0;i++){ //one-by-one truncate the constraint from the belief
          TruncateGaussian(__b, __B, cdir[i], coff(i));
          //plotTruncationLine(cdir[i], coff[i]);  plotCovariance(__b, __B);  plot();
        }
        //compute the EP message and 'add' it to the task message
        inverse_SymPosDef(__Binv, __B);
        R[t]() += __Binv - Binv[t];
        r[t]() += __Binv * __b - Binv[t]*b[t];

        //recompute (b, B);
        Binv[t] = Sinv[t] + Vinv[t] + R[t];
        Lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
        //cout <<"b\n" <<b[t] <<endl <<B[t] <<endl;

        //sys->displayState(b[t], &Binv[t], STRING("AICO kinematic (after truncation) t=" <<t));
      }
#endif

#ifndef TightMode
      //decide on \hat q
      if(sweep){ // || !useFwdMessageAsInitialQhat){
        double maxdiff = maxDiff(qhat[t], b[t]);
        if(maxdiff>.01){
          double a=.01/maxdiff;
          qhat[t]()=(1.-a)*qhat[t] + a*b[t];
        }else{
          if(convergenceRate) qhat[t]()=((double)1.-convergenceRate)*qhat[t] + convergenceRate*b[t];
          else qhat[t]()=b[t];
        }
      }
#else
      //update qhat
      if(dt==1){
        if(convergenceRate) qhat[t]()=((double)1.-convergenceRate)*qhat[t] + convergenceRate*b[t];
        else qhat[t]()=b[t];
      }
#endif

      //decide whether to repeat this time slice
      if(sweep && repeatThreshold && t!=T){
        double off=sqrDistance(b[t], qhat[t]);  //sqrDistance(W, b[t], qhat[t]);
        if(off>repeatThreshold){
          //cout <<t <<" REPEAT: off=" <<off <<" (repeatCount=" <<repeatCount <<")" <<endl;
          if(repeatCount<20){
            t-=dt;
            repeatCount++;
          }else{
            cout <<" ** no convergence! ** (skipping repeat) at t=" <<t <<endl;
            repeatCount=0;
          }
        }else{
          repeatCount=0;
        }
      }
    } //loop t
    sweep++;
#ifdef TightMode
    b=qhat;
#endif

  }//loop over dt in {-1, 1}
    
#if 1
  q = b;
#else
  getControlledTrajectory(q, *this);
#endif
  double diff = -1.;
  if(q_old.N==q.N) diff=maxDiff(q_old, q);
  
  //display or evaluate
  MT::timerPause();
  if(sys->os){
#ifdef NIKOLAY
    *sys->os <<std::setw(3) <<sweep <<"  " <<MT::timerRead(false);
#else
    *sys->os <<"AICOk(" <<scale <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" diff " <<diff;
#endif
   cost = sys->analyzeTrajectory(b, display>0);
  }
  
  if(display){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_kinematic - sweep " <<sweep)); //&Binv
  }
  MT::timerResume();

  return diff;
}

/// Approximate Inference Control (AICO) in the general (e.g. dynamic) case
double AICO_clean::stepClean(){
  //CHECK(sys->dynamic, "assumed dynamic SOC abstraction");
  uint T=sys->get_T();
  uint t;
  int dt;

  //get state info for t=0
  arr q0;
  sys->get_x0(q0);
  if(sys->dynamic){  CHECK(q0.N==2*sys->qDim(), "");  }else{  CHECK(q0.N==sys->qDim(), "");  }
  
  s[0]=q0;      Sinv[0].setDiag(1e10);
  b[0]=q0;      Binv[0].setDiag(1e10);
  qhat[0]=q0;
  sys->setx(q0);
  sys->getQ(Q[0](), 0);
  sys->getHinv(Hinv[0](), 0);
  if(!sys->dynamic) sys->getWinv(Winv[0](), 0);
  sys->getDynamics(A[0](), tA[0](), Ainv[0](), invtA[0](), a[0](), B[0](), tB[0](), 0);

  //OPTIONAL: take account of optional externally given bwd messages
  if(useBwdMsg){
    v[T] = bwdMsg_v;
    Vinv[T] = bwdMsg_Vinv;
  }

  //remember the old trajectory and old qhat
  double cost_old = cost;
  arr b_old(b);
  arr q_old(q);
  arr qhat_old(qhat);
  arr s_old=s, Sinv_old=Sinv, v_old=v, Vinv_old=Vinv, r_old=r, R_old=R;

  //damping
  arr Dinv;
  Dinv.setDiag(damping, dampingReference.d1);
               
  //MT::timerStart();
  uint repeatCount=0;

  for(t=1, dt=1;t>0;){ //start at t=1 going forward...
      
    //compute (s, S)
    if(dt==1 && !repeatCount) updateFwdMessage(t);  //only on fwd pass and non-repeats

    //compute (v, V)
    if(dt==-1 && !repeatCount) updateBwdMessage(t); //only on bwd pass and non-repeats

    //set the simulator state to qhat
    if(useFwdMessageAsQhat) qhat[t]()=s[t];

    if(true){ //dt==1){ //always, or only on forward?
      countSetq++;
      sys->setx(qhat[t]);
      
      //compute system matrices
      sys->getQ(Q[t](), t);
      sys->getHinv(Hinv[t](), t);
      if(!sys->dynamic) sys->getWinv(Winv[t](), t);
      sys->getDynamics(A[t](), tA[t](), Ainv[t](), invtA[t](), a[t](), B[t](), tB[t](), t);
      
      //compute (r, R)
      sys->getTaskCosts(R[t](), r[t](), qhat[t].sub(0, sys->qDim()-1), t, &rhat(t));
      //rhat(t) -= scalarProduct(R[t], qhat[t], qhat[t]) - 2.*scalarProduct(r[t], qhat[t]);
    }
    
    //compute (b, B);
    if(damping && dampingReference.N){
      Binv[t] = Sinv[t] + Vinv[t] + R[t] + Dinv;
      lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t] + Dinv*dampingReference[t]);
    }else{
      Binv[t] = Sinv[t] + Vinv[t] + R[t];
      lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
    }
    //cout <<"b\n" <<b[t] <<endl <<B[t] <<endl;

    //decide on a new \hat q
    if(!useFwdMessageAsQhat){
      qhat[t]()=b[t];
    }

    if(t==T && dt==1){ //go backward again
      dt=-1;
      useFwdMessageAsQhat=false;
    }else{
      t+=dt;
    }

  }
  sweep += 2;
  
  if(sys->dynamic) soc::getPositionTrajectory(q, b); else q=b;

  double diff = -1;
  if(q_old.N==q.N) diff=maxDiff(q_old, q);
  cost = sys->analyzeTrajectory(b, display>0);
  
  //-- analyze whether to reject the step and increase damping (to guarantee convergence)
  if(sweep>3 && damping){
    if(cost>cost_old){
      damping *= 10.;
      dampingReference=b_old;
      //cout <<" AICOd REJECT: cost=" <<cost <<" cost_old=" <<cost_old <<endl;
      b = b_old;
      q = q_old;
      qhat = qhat_old;
      cost = cost_old;
      s=s_old; Sinv=Sinv_old; v=v_old; Vinv=Vinv_old; r=r_old; R=R_old;
    }else{
      damping /= 5.;
      dampingReference=b;
      //cout <<" AICOd ACCEPT" <<endl;
    }
  }else{
    dampingReference=b;
  }
  
  //-- display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"AICOclean(" <<scale <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" diff " <<diff <<" damp " <<damping;
    //sys->analyzeTrajectory(b, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_clean - iteration " <<sweep));
  }
  MT::timerResume();

  return diff;
}

//==============================================================================


/// Approximate Inference Control (AICO) in the general (e.g. dynamic) case
double AICO_clean::stepDynamic(){
  //CHECK(sys->dynamic, "assumed dynamic SOC abstraction");
  uint T=sys->get_T();
  uint t;
  int dt;

  //get state info for t=0
  arr q0;
  sys->get_x0(q0);
  if(sys->dynamic){
    CHECK(q0.N==2*sys->qDim(), "");
  }else{
    CHECK(q0.N==sys->qDim(), "");
  }
  s[0]=q0;      Sinv[0].setDiag(1e10);
  b[0]=q0;      Binv[0].setDiag(1e10);
  qhat[0]=q0;
  sys->setx(q0);
  sys->getQ(Q[0](), 0);
  sys->getHinv(Hinv[0](), 0);
  if(!sys->dynamic) sys->getWinv(Winv[0](), 0);
  sys->getDynamics(A[0](), tA[0](), Ainv[0](), invtA[0](), a[0](), B[0](), tB[0](), 0);

  //OPTIONAL: take account of optional externally given bwd messages
  if(useBwdMsg){
    v[T] = bwdMsg_v;
    Vinv[T] = bwdMsg_Vinv;
  }

  //remember the old trajectory and old qhat
  double cost_old = cost;
  arr b_old(b);
  arr q_old(q);
  arr qhat_old(qhat);
  arr s_old=s, Sinv_old=Sinv, v_old=v, Vinv_old=Vinv, r_old=r, R_old=R;

  //damping
  arr Dinv;
  Dinv.setDiag(damping, dampingReference.d1);
               
  //MT::timerStart();
  uint repeatCount=0;

  for(t=1, dt=1;t>0;){ //start at t=1 going forward...
      
    //compute (s, S)
    if(dt==1 && !repeatCount) updateFwdMessage(t);  //only on fwd pass and non-repeats

    //compute (v, V)
    if(dt==-1 && !repeatCount) updateBwdMessage(t); //only on bwd pass and non-repeats

    //set the simulator state to qhat
    if(useFwdMessageAsQhat) qhat[t]()=s[t];
    if(true){ //dt==1){
      countSetq++;
      sys->setx(qhat[t]);
      
      //compute system matrices
      sys->getQ(Q[t](), t);
      sys->getHinv(Hinv[t](), t);
      if(!sys->dynamic) sys->getWinv(Winv[t](), t);
      sys->getDynamics(A[t](), tA[t](), Ainv[t](), invtA[t](), a[t](), B[t](), tB[t](), t);
      
      //compute (r, R)
      sys->getTaskCosts(R[t](), r[t](), qhat[t].sub(0, sys->qDim()-1), t, &rhat(t));
      //rhat(t) -= scalarProduct(R[t], qhat[t], qhat[t]) - 2.*scalarProduct(r[t], qhat[t]);
    }
    
    //compute (b, B);
    if(damping && dampingReference.N){
      Binv[t] = Sinv[t] + Vinv[t] + R[t] + Dinv;
      lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t] + Dinv*dampingReference[t]);
    }else{
      Binv[t] = Sinv[t] + Vinv[t] + R[t];
      lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
    }
    //cout <<"b\n" <<b[t] <<endl <<B[t] <<endl;

    //decide on a new \hat q
    if(!useFwdMessageAsQhat){
#if 0
      if(convergenceRate) qhat[t]()=((double)1.-convergenceRate)*qhat[t] + convergenceRate*b[t];
      else qhat[t]()=b[t];
#else
      if(maxStep){
        arr delta = b[t]-qhat[t];
        if(convergenceRate) delta *= convergenceRate;
        double len = norm(delta);
        if(len>maxStep){
          qhat[t]() += (maxStep/len)*delta;
        }else qhat[t]() += delta;
      }else qhat[t]() = b[t];
#endif
    }

    //decide whether to repeat this time slice
    if(repeatThreshold && t!=T){ //&& sweep
      //double off=sqrDistance(Q, b[t], qhat[t]);
      double off=sqrDistance(b[t], qhat[t]);
      if(off>repeatThreshold){
        //cout <<t <<" REPEAT: off=" <<off <<" (repeatCount=" <<repeatCount <<")" <<endl;
        if(repeatCount<20){
          t-=dt;
          repeatCount++;
        }else{
          cout <<" ** no convergence! ** (skipping repeat) at t=" <<t <<endl;
          repeatCount=0;
        }
      }else{
        repeatCount=0;
      }
    }

    if(t==T && dt==1){ //go backward again
      dt=-1;
      useFwdMessageAsQhat=false;
    }else{
      t+=dt;
    }

  }
  sweep += 2;
  
  if(sys->dynamic) soc::getPositionTrajectory(q, b); else q=b;

  double diff = -1;
  if(q_old.N==q.N) diff=maxDiff(q_old, q);
  cost = sys->analyzeTrajectory(b, display>0);
  //damping = 1e1;
  if(sweep>3 && damping){
    if(cost>cost_old){
      damping *= 10.;
      dampingReference=b_old;
      //cout <<" AICOd REJECT: cost=" <<cost <<" cost_old=" <<cost_old <<endl;
      b = b_old;
      q = q_old;
      qhat = qhat_old;
      cost = cost_old;
      s=s_old; Sinv=Sinv_old; v=v_old; Vinv=Vinv_old; r=r_old; R=R_old;
    }else{
      damping /= 5.;
      dampingReference=b;
      //cout <<" AICOd ACCEPT" <<endl;
    }
  }else{
    dampingReference=b;
  }
  
  //display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"AICOd(" <<scale <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" diff " <<diff <<" damp " <<damping;
    //sys->analyzeTrajectory(b, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_dynamic - iteration " <<sweep));
  }
  MT::timerResume();

  return diff;
}

//==============================================================================

double AICO_clean::stepGaussNewton(){
  uint T=sys->get_T();
  uint t;
  int dt;

  //get state info for t=0
  arr q0;
  if(sys->dynamic) sys->getqv0(q0); else sys->getq0(q0);
  s[0]=q0;      Sinv[0].setDiag(1e10);
  b[0]=q0;      Binv[0].setDiag(1e10);
  qhat[0]=q0;
  if(sys->dynamic) sys->setx(q0); else sys->setq(q0);
  sys->getQ(Q[0](), 0);
  if(!sys->dynamic) sys->getWinv(Winv[0](), 0);
  sys->getHinv(Hinv[0](), 0);
  sys->getDynamics(A[0](), tA[0](), Ainv[0](), invtA[0](), a[0](), B[0](), tB[0](), 0);

  //OPTIONAL: take account of optional externally given bwd messages
  if(useBwdMsg){
    v[T] = bwdMsg_v;
    Vinv[T] = bwdMsg_Vinv;
    //Vinv[T].setDiag(1e1); //HACK!
  }

  //remember the old trajectory and old qhat
  double cost_old = cost;
  arr b_old(b);
  arr q_old(q);
  arr qhat_old(qhat);
  arr s_old=s, Sinv_old=Sinv, v_old=v, Vinv_old=Vinv, r_old=r, R_old=R;

  struct LocalCostFunction:public GaussNewtonCostFunction{
    uint t;
    soc::SocSystemAbstraction* sys;
    AICO_clean* aico;
    bool reuseOldCostTerms, noBwdMsg;
    
    void calcTermsAt(const arr &x){
      //all terms related to task costs
      if(reuseOldCostTerms){
        phi = aico->phiBar(t);  J = aico->JBar(t);
        reuseOldCostTerms=false;
      }else{
        countSetq++;
        if(sys->dynamic) sys->setx(x); else sys->setq(x);
        sys->getTaskCostTerms(phi, J, x, t);
        aico->phiBar(t) = phi;  aico->JBar(t) = J;
      }
      
      //store cost terms also as R, r matrices
      aico->R[t] =   ~(aico->JBar(t)) * (aico->JBar(t));
      aico->r[t] = - ~(aico->JBar(t)) * (aico->phiBar(t));
      aico->r[t]() += aico->R[t] * x;
      
      //add the damping
      if(aico->damping && aico->dampingReference.N){
        J.append(   diag(aico->damping, x.N) );
        phi.append( aico->damping*(x-aico->dampingReference[t]) );
      }
      
      arr M;

      //add the forward message
      lapack_cholesky(M, aico->Sinv[t]); //ensures Sinv = ~M*M
      phi.append( M*(x-aico->s[t]) );
      J  .append( M );
      
      if(noBwdMsg) return;
      //add the mackward message
      lapack_cholesky(M, aico->Vinv[t]);
      phi.append( M*(x-aico->v[t]) );
      J  .append( M );
    }
  } f;
  
  for(t=1, dt=1;t>0;){ //start at t=1 going forward...

    //compute (s, S)
    if(dt==1) updateFwdMessage(t);  //only on fwd pass
      
    //compute (v, V)
    if(dt==-1) updateBwdMessage(t); //only on bwd pass
        
    if(useFwdMessageAsQhat){
      qhat[t]()=s[t];
      countSetq++;
      if(sys->dynamic) sys->setx(qhat[t]); else sys->setq(qhat[t]);
      f.sys=sys;  f.aico=this;  f.t=t;  f.reuseOldCostTerms=false;  f.noBwdMsg=true;
      f.calcTermsAt(qhat[t]);
    }else{
      f.sys=sys;  f.aico=this;  f.t=t;  f.reuseOldCostTerms=true;   f.noBwdMsg=false;
      //if(!sweep)
      f.reuseOldCostTerms=false;
      if(!repeatThreshold) HALT("need to set repeatThreshold for AICO_gaussNewton")
      GaussNewton(qhat[t](), repeatThreshold, f, 5);
    }
    
    //compute system matrices
    sys->getQ(Q[t](), t);
    if(!sys->dynamic) sys->getWinv(Winv[t](), t);
    sys->getHinv(Hinv[t](), t);
    sys->getDynamics(A[t](), tA[t](), Ainv[t](), invtA[t](), a[t](), B[t](), tB[t](), t);
    //if(t<T){
    //  arr tmp;
    //  sys->getTransitionCostTerms(Psi[t+1](), tmp, tmp, qhat[t], qhat[t+1], t+1);
    //}
    
    //*
    //compute (r, R) -- is done in LocalCostFunction
    //arr Rt, rt;
    //sys->getTaskCosts(Rt, rt, qhat[t].sub(0, sys->qDim()-1), t);
    //R[t] = Rt; r[t] = rt;
    
    //compute (b, B);
    Binv[t] = Sinv[t] + Vinv[t] + R[t];
    lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
    //*/
    
    if(t==T && dt==1){ //go backward again
      dt=-1;
      useFwdMessageAsQhat=false;
    }else{
      t+=dt;
    }
  }
  sweep += 2;
  
  if(sys->dynamic) soc::getPositionTrajectory(q, b); else q=b;
  
  double diff = -1;
  if(q_old.N==q.N) diff = maxDiff(q_old, q);
  cost = sys->analyzeTrajectory(b, display>0);
  //double tc=0.; for(uint k=0;k<phiBar.N;k++) tc+=sumOfSqr(phiBar(k));
  ///cout <<"internal cost: taskC= " <<tc <<" ctrlC= " <<sumOfSqr(Psi) <<endl;
  if(sweep>3 && damping){
    if(cost>cost_old){
      damping *= 10.;
      dampingReference=qhat_old;
      //cout <<" AICOgn REJECT: cost=" <<cost <<" cost_old=" <<cost_old <<endl;
      b = b_old;
      q = q_old;
      qhat = qhat_old;
      cost = cost_old;
      s=s_old; Sinv=Sinv_old; v=v_old; Vinv=Vinv_old; r=r_old; R=R_old;
    }else{
      damping /= 5.;
      dampingReference=qhat;
      //cout <<" AICOgn ACCEPT" <<endl;
    }
  }else{
    dampingReference=qhat;
  }    
  
  //display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"AICOgn(" <<T <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<diff <<" damp " <<damping;
    //sys->analyzeTrajectory(b, display>0);
    //sys->costChecks(b);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_GaussNewton - iteration " <<sweep));
  }
  MT::timerResume();

  return diff;
}


double AICO_clean::stepMinSum(){
  if(sys->os){
    *sys->os <<"AICOgn(" <<sys->get_T() <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" before";
    cost = sys->analyzeTrajectory(b, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_GaussNewton - iteration " <<sweep));
  }
  
  struct AICO_clean_MinSum:MinSumGaussNewton{
    soc::SocSystemAbstraction* sys;
    AICO_clean* aico;
    arr x0;
    void set(uint T, uint n){
      if(!sys->dynamic) x=aico->q; else soc::getPhaseTrajectory(x, aico->q, sys->getTau());
      if(sys->dynamic) sys->getqv0(x[0]()); else sys->getq0(x[0]());
      
      uint i;
      for(i=0;i<=T;i++){
        E.append(TUP(i, i));
        if(i>0) E.append(TUP(i-1, i));
        if(i<T) E.append(TUP(i+1, i));
      }
      E.reshape(E.N/2, 2);
      del.resize(T+1);
      for(i=0;i<E.d0;i++) del(E(i, 1)).append(i);
      cout <<"E=" <<E <<"del=" <<del <<endl;
      
      clamped.resize(T+1);
      clamped=false;
      clamped(0)=true;
    }
    void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){
      CHECK(j<=i, "");
      if(i==j){//task potentials
        sys->setx(x_i);
        sys->getTaskCostTerms(psi, psiI, x_i, i);
      }else{ //transition potentials
        if(sys->dynamic){
          arr Hinv, A, a, B, Q, W, Winv, M;
          sys->getHinv(Hinv, j);
          sys->getDynamics(A, a, B, j);
          sys->getQ(Q, j);
          psi = x_i - (A*x_j+a);
          Winv = B*Hinv*~B + Q;
          inverse_SymPosDef(W, Winv);
          lapack_cholesky(M, W);
          psi = M*psi;
          psiI = M;
          psiJ = -M*A;
        }else{
          arr W, M;
          sys->getW(W, j);

          psi = x_i - x_j;
          lapack_cholesky(M, W);
          psi = M*psi;
          psiI = M;
          psiJ = -M;
        }
      }
    }
  };

  static AICO_clean_MinSum f;
  static bool first=true;
  
  if(first){
    f.sys=sys;
    f.aico=this;
    f.set(sys->get_T(), b.d1);
    f.tolerance = 1e-3;
    f.maxStep = 1e-1;
    if(sys->dynamic) sys->getqv0(f.x0); else sys->getq0(f.x0);
    f.init();
    first=false;
  }
  
  //remember the old trajectory and old qhat
  double diff = -1.;
  arr q_old;
  if(sys->dynamic)  soc::getPositionTrajectory(q_old, b);  else  q_old=b;

  f.step(1);

  b=f.x;
  if(sys->dynamic)  soc::getPositionTrajectory(q, b);  else  q=b;
  if(q_old.N==q.N) diff=maxDiff(q_old, q);
  
  //display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"AICOgn(" <<sys->get_T() <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<diff;
    cost = sys->analyzeTrajectory(b, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_GaussNewton - iteration " <<sweep));
  }
  MT::timerResume();

  return diff;
}
