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

#ifndef MT_soc_SI_h
#define MT_soc_SI_h

#include "soc.h"
#include "plot.h"

/** \brief very preliminary... */
struct SocSystem_Analytical:public virtual soc::SocSystemAbstraction {
  uint T;
  arr x0, x1, x, W;
  double prec;
  arr obstacles;
  
  SocSystem_Analytical(){}
  virtual ~SocSystem_Analytical(){}
  
  //initialization methods
  void initKinematic(uint dim, uint trajectory_length, double w, double endPrec){
    x0.resize(dim); x0.setZero();
    x1=x0; x1(0)=1.;
    x=x0;
    W.setDiag(w, x.N);
    prec=endPrec;
    T=trajectory_length;
    obstacles.resize(2, x.N);
    obstacles(0, 0)=.3; obstacles(0, 1)=.05;
    obstacles(1, 0)=.7; obstacles(1, 1)=-.05;
    dynamic=false;
    //os = &cout;
  }
  void initDynamic(uint dim, double trajectory_time, uint trajectory_steps, arr*H=NULL){
    NIY;
    dynamic=true;
  }
  
  //implementations of virtual methods
  uint nTime(){ return T; }
  uint nTasks(){ return 2; }
  uint qDim(){ return x0.N; }
  uint uDim(){ return x0.N; }
  uint yDim(uint i){ if(!i) return x0.N; else return 1; }
  void getq0(arr& q){ q=x0; }
  void getv0(arr& v){ NIY; }
  void getqv0(arr& q_){ NIY; }
  void getqv0(arr& q, arr& qd){ NIY; }
  bool isDynamic(){ return false; }
  void setq(const arr& q){ x=q; }
  void setqv(const arr& q_){ NIY; }
  void setqv(const arr& q, const arr& qd){ NIY; }
  void setq0AsCurrent(){ NIY; }
  void geth(arr& h){ NIY; }
  void getW(arr& _W){ _W=W; }
  void getH(arr& H){ NIY; }
  void getQ(arr& Q){ NIY; }
  bool isConditioned(uint i, uint t){ NIY; if(!i && t==T-1) return true;  return false; }
  bool isConstrained(uint i, uint t){ NIY; if(i==1) return true;  return false; }
  const char* taskName(uint i){ NIY; return "task"; }
  void getPhi(arr& phiq_i, uint i){
    NIY;
    if(i==1){
      phiq_i.resize(1);
      for(uint i=0; i<obstacles.N; i++){
        phiq_i(0) += norm(x-obstacles[i]);
      }
    }
  }
  
  void getProcess(arr& A, arr& a, arr& B);
  double getTaskCosts(arr& R, arr& r, uint t, const arr& qt);
  void getConstraints(arr& c, arr& coff, uint t, const arr& qt);
  
  void displayState(const arr& q, const arr *Qinv, const char *text=NULL);
  void displayTrajectory(const arr& q, const arr *Qinv, int steps, const char *tag=NULL);
};

void SocSystem_Analytical::getProcess(arr& A, arr& a, arr& B){
  uint N=x.N;
  A.setDiag(1., N);
  B.setDiag(1., N);
  a.resize(N); a.setZero();
}

double SocSystem_Analytical::getTaskCosts(arr& R, arr& r, uint t, const arr& qt){
  uint N=x.N;
  R.resize(N, N); R.setZero();
  r.resize(N);   r.setZero();
  double C=0.;
  
#ifndef USE_TRUNCATION //potentials for collision cost
  arr J(1, qt.N), phiHatQ(1);
  J.setZero();
  phiHatQ.setZero();
  for(uint i=0; i<obstacles.d0; i++){
    double margin = .1;
    double d = (1.-norm(x-obstacles[i])/margin);
    if(d<0) continue;
    phiHatQ(0) += d*d;
    J += ((double)2.*d/margin)*(obstacles[i]-x)/norm(x-obstacles[i]);
  }
  J.reshape(1, J.N);
  arr tJ, target(1);
  target=(double)0.;
  transpose(tJ, J);
  double colprec = (double)5e2;
  C += colprec*sqrDistance(target, phiHatQ);
  R += colprec*tJ*J;
  r += colprec*tJ*(target - phiHatQ + J*qt);
#endif
  
  if(t!=T-1) return C;
  R.setDiag(1.);
  r = x1;
  R *= prec;
  r *= prec;
  C += prec*sqrDistance(x1, x);
  return C;
}

void SocSystem_Analytical::getConstraints(arr& cdir, arr& coff, uint t, const arr& qt){
  cdir.clear();
  coff.clear();
#ifndef USE_TRUNCATION
  return;
#endif
  uint i, M=obstacles.d0;
  arr d;
  
#if 0 //direct and clean way to do it -- but depends simple scenario
  cdir.resize(M, x.N);
  coff.resize(M);
  for(i=0; i<M; i++){
    cdir[i] = qt-obstacles[i];
    coff(i) = scalarProduct(cdir[i], obstacles[i]);
  }
#elif 1 //assume that the task vector is a list of scalars, each constrained >0
  arr J, y;
  for(i=0; i<M; i++){
    double haty = norm(x-obstacles[i]);
    if(haty>.5) continue; //that's good enough -> don't add the constraint
    J = (x-obstacles[i])/norm(x-obstacles[i]);
    coff.append(-haty + scalarProduct(J, x));
    cdir.append(J);
  }
  cdir.reshape(coff.N, x.N);
  coff.reshape(coff.N);
#else //messy: try to combine all constraints into a single scalar, doesn't really work...
  //first compute squared collision meassure...
  arr J(1, qt.N), phiHatQ(1);
  J.setZero();
  phiHatQ.setZero();
  for(i=0; i<obstacles.d0; i++){
    double margin = .25;
    double d = 1.-norm(x-obstacles[i])/margin;
    //if(d<0) continue;
    //phiHatQ(0) += d*d;
    //J += (2.*d/margin)*(obstacles[i]-x)/norm(x-obstacles[i]);
    phiHatQ(0) += d;
    J += (1./margin)*(obstacles[i]-x)/norm(x-obstacles[i]);
  }
  //...then add a single constraint
  if(phiHatQ(0)>0.){ //potential violation, else discard
    cdir.append(-J);
    coff.append(phiHatQ-scalarProduct(J, x)-1.);
    cdir.reshape(1, x.N);
    coff.reshape(1);
  }
#endif
}

void SocSystem_Analytical::displayState(const arr& q, const arr *Qinv, const char *text){
  cout <<"gnuplot state display " <<text <<endl;
  plotGnuplot();
  plotClear();
  plotPoints(obstacles);
  plotPoint(q);
  arr C;
  inverse_SymPosDef(C, (*Qinv));
  plotCovariance(q, C);
  plot();
}

void SocSystem_Analytical::displayTrajectory(const arr& q, const arr *Qinv, int steps, const char *tag){
  cout <<"gnuplot trajectory display " <<tag <<endl;
  plotGnuplot();
  plotClear();
  plotPoints(obstacles);
  plotLine(q);
  arr C;
  for(uint t=0; t<q.d0; t+=1){
    inverse_SymPosDef(C, (*Qinv)[t]);
    plotCovariance(q[t], C);
  }
  plot();
}



#endif
