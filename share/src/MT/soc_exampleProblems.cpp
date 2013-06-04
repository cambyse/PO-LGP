/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
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
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

/**
 * @file
 * @ingroup group_soc
 */

#include "soc_exampleProblems.h"
#include <Gui/plot.h>

ControlledSystem_PointMass::ControlledSystem_PointMass(){
  T = MT::getParameter<uint>("T",10);
  tau = 1./get_T();
  x0 = ARR(1., 1.);
  x_target = ARR(0., 1.);
  prec = 1e2;
}

ControlledSystem_PointMass::~ControlledSystem_PointMass(){
}

void ControlledSystem_PointMass::getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a, arr& B, arr& Bt, arr& Q, uint t){
  A.setDiag(1., 2); A(0,1) = tau;
  B.resize(2,1); B(0,0)=tau*tau/2.; B(1,0)=tau;
  a.resize(2); a.setZero();
  if(&At){ At.setDiag(1.,2); At(1,0) = tau; }
  if(&Ainv){  Ainv.setDiag(1.,2); Ainv(1,0) = -tau;  }
  if(&Ainvt){  Ainvt.setDiag(1.,2); Ainvt(0,1) = -tau;  }
  if(&Bt){ transpose(Bt, B); }
  Q.setDiag(1e-6,2);
}

void ControlledSystem_PointMass::getControlCosts(arr& H, arr& Hinv, uint t){
  if(&H) H.setDiag(tau,1.);
  if(&Hinv) Hinv.setDiag(1./tau,1.);
}

void ControlledSystem_PointMass::getTaskCosts(arr& phi, arr& phiJ, uint t){
  if(t!=0 && t!=get_T()){
    phi.resize(x.N); phi.setZero();
    if(&phiJ){ phiJ.resize(x.N,x.N); phiJ.setZero(); }
    return;
  }
  if(t==0){
    phi = prec*(x-x0);
    if(&phiJ) phiJ.setDiag(prec,x.N);
  }
  if(t==get_T()){
    phi = prec*(x-x_target);
    if(&phiJ) phiJ.setDiag(prec,x.N);
  }
}


void ControlledSystem_PointMass::displayCurrentState(const char* title, bool pause, bool reportOnTasks){
  cout <<"gnuplot " <<MT_HERE <<title <<endl;
  plotGnuplot();
  plotClear();
  plotPoint(x);
  plot(pause);
}

void ControlledSystem_PointMass::getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t){
  dims.resize(2); dims=1;
  names.resize(2);
  names(0)="position_cost";
  names(1)="velocity_cost";
}
