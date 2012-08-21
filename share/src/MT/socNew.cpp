#include "socNew.h"

double ControlledSystem::getTaskCosts(arr& R, arr& r, uint t, double* rhat){
  arr phi, J;
  getTaskCosts(phi, J, t);
  //x is the curren state!
  if(!phi.N){ R=zeros(get_xDim()); r=zeros(TUP(get_xDim())); return 0.; }
  innerProduct(R, ~J, J);
  innerProduct(r, ~J, J*getx() - phi);
  if(rhat) *rhat = sumOfSqr(J*getx() - phi);
  return sumOfSqr(phi);
}

void getTransitionCostTerms(ControlledSystem& sys, bool dynamic, arr& Psi, arr& J0, arr& J1, const arr& x0, const arr& x1, uint t){
  if(!dynamic){
    arr H, M;
    sys.getControlCosts(H, NoArr, t);
    Psi = x1 - x0;
    lapack_cholesky(M, H);
    Psi = M*Psi;
    if(&J0) J0 = -M;
    if(&J1) J1 = M;
  }else{
    arr Hinv, A, a, B, Q, W, Winv, M;
    sys.getDynamics(A, a, B, Q, t);
    sys.getControlCosts(NoArr, Hinv, t);
    Psi = x1 - (A*x0+a);
    for(uint i=0;i<Q.d0/2;i++)
      Q(i,i) += 1e-4; //allow for noise in the position transitions!
    Winv = B*Hinv*~B + Q;
    inverse_SymPosDef(W, Winv);
    lapack_cholesky(M, W);
    Psi = M*Psi;
    if(&J0) J0 = -M*A;
    if(&J1) J1 = M;
  }
};

double analyzeTrajectory(ControlledSystem& sys, const arr& x, bool plot, std::ostream* os){
  uint t, T=sys.get_T();
  CHECK(x.nd==2 && x.d0==T+1 && x.d1==sys.get_xDim(), "");
 
  arr taskC(T+1);  taskC.setZero();
  arr ctrlC(T+1);  ctrlC.setZero();
  for(t=0; t<=T; t++){
    sys.setx(x[t]);
    
    arr phi,psi;
    sys.getTaskCosts(phi, NoArr, t);
    taskC(t)=sumOfSqr(phi);

    if(t<T){
      getTransitionCostTerms(sys, true, psi, NoArr, NoArr, x[t], x[t+1], t);
      ctrlC(t)=sumOfSqr(phi);
    }else{
      ctrlC(t)=0.;
    }
  }
  if(plot){
    std::ofstream fil;
    MT::open(fil, "z.trana");
    for(t=0; t<=T; t++){
      fil <<"time_step " <<t
      <<"  ctrlC " <<ctrlC(t)
      <<"  taskC " <<taskC(t);
      /*      fil <<"  taskCi "; taskCi[t].writeRaw(fil);
      fil <<"  taskDx "; taskDx[t].writeRaw(fil);
      fil <<"  taskDv "; taskDv[t].writeRaw(fil);*/
      fil <<"  x "; x[t].writeRaw(fil);
      fil <<endl;
    }
    MT::String cmd;
    cmd <<"set style data linespoints\n";
    cmd <<"plot 'z.trana' us 0:4 title 'ctrlC','z.trana' us 0:6 title 'taskC'";
    /*    if(!dynamic){
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<8+i <<" title '" <<taskName(i) <<"'";
    }else{
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<9+m+i <<" title '" <<taskName(i) <<"_q'";
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<10+2*m+i <<" title '" <<taskName(i) <<"_v'";
      }*/
    gnuplot(cmd);
  }

  double taskCsum=sum(taskC), ctrlCsum=sum(ctrlC);
  if(os) *os
    <<"  task-cost " <<taskCsum
    <<"  control-cost " <<ctrlCsum
    <<"  total-cost " <<taskCsum+ctrlCsum <<endl;
  return taskCsum+ctrlCsum;
}

void displayTrajectory(ControlledSystem& sys, const arr& x, const arr *Binv, int steps, const char *tag){
  uint k, t, T=sys.get_T();
  if(!sys.gl || !steps) return;
  uint num;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(k=0; k<=(uint)num; k++){
    t = k*T/num;
    sys.setx(x[t]);
    sys.displayCurrentState(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p, steps==-1);
  }
  if(steps==1)
    sys.displayCurrentState(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p, true);
}
