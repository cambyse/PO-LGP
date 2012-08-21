#include "soc_exampleProblems.h"
#include "plot.h"

ControlledSystem_PointMass::ControlledSystem_PointMass(){
  T=100;
  tau = .01;
  x0 = ARR(0., 0.);
  x_target = ARR(1., 0.);
  prec = 1e2;
}

ControlledSystem_PointMass::~ControlledSystem_PointMass(){
}

void ControlledSystem_PointMass::getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a, arr& B, arr& Bt, uint t){
  A.setDiag(1., 2); A(0,1) = tau;
  B.resize(1,2); B(0,0)=0.; B(0,1)=1.;
  a.resize(2); a.setZero();
  if(&At){ At.setDiag(1.,2); At(1,0) = tau; }
  if(&Ainv){  Ainv.setDiag(1.,2); Ainv(1,0) = -tau;  }
  if(&Ainvt){  Ainvt.setDiag(1.,2); Ainvt(0,1) = -tau;  }
  if(&Bt){ Bt.resize(2,1); B(0,0)=0.; B(1,0)=1.; }
}

void ControlledSystem_PointMass::getControlCosts(arr& H, arr& Hinv, uint t){
  if(&H) H.setDiag(tau,1);
  if(&Hinv) Hinv.setDiag(1./tau,1);
}

void ControlledSystem_PointMass::getTaskCosts(arr& phi, arr& phiJ, uint t){
  if(t!=T){ phi.clear(); phiJ.clear(); return; }
  if(t==T){
    phi = prec*(x-x_target);
    phiJ.setDiag(prec,x.N);
  }
}


void ControlledSystem_PointMass::displayCurrentState(bool reportOnTasks){
  cout <<"gnuplot " <<MT_HERE <<endl;
  plotGnuplot();
  plotClear();
  plotPoint(x);
  plot();
}

void ControlledSystem_PointMass::getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names){
  dims.resize(2); dims=1;
  names.resize(2);
  names(0)="position_cost";
  names(1)="velocity_cost";
}
