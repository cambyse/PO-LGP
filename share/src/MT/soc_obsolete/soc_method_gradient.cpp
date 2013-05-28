/*  ---------------------------------------------------------------------
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
    -----------------------------------------------------------------  */

#include "soc.h"
#include "ors.h"
#include "optimization.h"
#ifdef MT_HSL
#  include "hslModule.h"
#endif

double attractorCostAndGradient(soc::SocSystemAbstraction& soci, arr& dCdvia, arr& q, const ors::Spline& spline, const arr& via, bool costOnly){ NIY; }

//===========================================================================

struct SocProblem:public OptimizationProblem{
  uint count;
  arr q, dq;
  soc::SocSystemAbstraction *sys;
  ostream *os;
  OpenGL *gl;
  ors::Spline *spline;
  uint display;

  SocProblem(){ count=0; }
      
  double f(arr *grad, const arr& x, int i=-1){
    sys->getq0(x[0]());
    if(!spline) q=x; else q=spline->basis*x;
    //if(display) sys->displayTrajectory(q, NULL, display, STRING("gradient optimization -- iteration " <<-1));
    if(os) (*os) <<std::setw(3) <<count++ <<"  time " <<MT::timerRead();
    if(os!=&cout) cout <<'.' <<std::flush;
    double C;
    if(!grad){
      C=sys->totalCost(NULL, q);
    }else{
      C=sys->totalCost(&dq, q);
      if(!spline) *grad=dq; else *grad=spline->basis_trans*dq;
      (*grad)[0]()=0.;
    }
    //if(display) sys->analyzeTrajectory(q, display>0);
    return C;
  }

  /*
  double f_att(const arr &x, void *data){
    arr q0;
    ((soc::SocSystemAbstraction*)data)->getq0(q0);
    ((soc::SocSystemAbstraction*)data)->setq(q0);
    ((soc::SocSystemAbstraction*)data)->getPhi(x[0](), 0);
    double C;
    static arr dummy;
    soc::SocSystemAbstraction& soci=*((soc::SocSystemAbstraction*)data);
    if(os) (*os) <<std::setw(3) <<count++ <<"  time " <<MT::timerRead();
    C=attractorCostAndGradient(soci, dummy, q, *spline, x, true);
    if(gl){
      *plotClear();
      plotPoints(x);
      plotLine(x);*
      NIY;
      ((soc::SocSystemAbstraction*)data)->displayTrajectory(q, NULL, display, STRING("gradient optimization -- iteration " <<-1));
    }
    //return ((soc::SocSystemAbstraction*)data)->computeTotalCost(q);
    return C;
  }

  void df_att(arr &dx, const arr &x, void *data){
    //((soc::SocSystemAbstraction*)data)->getq0(x[0]());
    soc::SocSystemAbstraction& soci=*((soc::SocSystemAbstraction*)data);
    attractorCostAndGradient(soci, dx, q, *spline, x, false);
    dx[0]()=0.;
  }
  */
};

//===========================================================================

/// compute an optimized trajectory using gradient descent
void soc::gradientOptimization(SocSystemAbstraction& soci,
                                arr& q,
                                uint maxIterations,
                                uint spline_points,
                                uint spline_degree,
                                double stoppingTolerance,
                                bool checkGradient,
                                uint display){

  uint T=soci.get_T();
  
  MT::timerStart();
  
  //initialize trajectory
  CHECK(q.nd==2 && q.d0==T+1 && q.d1==soci.qDim(), "please initialize trajectory!");
  
  if(soci.os){
    *soci.os <<std::setw(3) <<-1 <<"  time " <<MT::timerRead(false);
    //soci.computeTotalCost(q);
    soci.analyzeTrajectory(q, display>0);
  }
  if(soci.gl){
    soci.displayTrajectory(q, NULL, display, STRING("gradient optimization -- iteration " <<-1));
  }
  
  bool task_spline=false;
  //if(gradient_method==Attractor) task_spline=true;

  ::ors::Spline spline;
  if(spline_points){
    uint K=spline_points;
    spline.setUniformNonperiodicBasis(T, K, spline_degree);
    if(!task_spline){
      spline.points.resize(K+1, soci.qDim());
      spline.points[0]() = q[0];
      spline.points[K]() = q[T];
      for(uint k=1;k<K;k++)
        spline.points[k]() = q[(uint)((double)k*T/K)];
    }else{
      spline.points.resize(K+1, soci.yDim(0));
      soci.setq(q[0]); soci.getPhi(spline.points[0](), 0);
      soci.setq(q[T]); soci.getPhi(spline.points[K](), 0);
      for(uint k=1;k<K;k++){
        soci.setq(q[(uint)((double)k*T/K)]); soci.getPhi(spline.points[k](), 0);
      }
    }
  }

  SocProblem problem;
  problem.sys= &soci;
  problem.os = soci.os;
  problem.gl = soci.gl;
  problem.display = display;
  
  arr x; //variable to be optimized
  if(spline_points){// || gradient_method==Attractor){
    x = spline.points;
    problem.spline = &spline;
  }else{
    x = q;
    problem.spline = NULL;
  }
  
  double fmin;
  ::Rprop rprop;
  rprop.init(1e-2);
  rprop.loop(x, problem, &fmin, stoppingTolerance, maxIterations);

  if(spline_points) q = spline.basis*x; else q=x;
}

//===========================================================================
