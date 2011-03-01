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

#include"algos.h"
// Runge Kutta

typedef void fd(arr& xd, const arr& x);
typedef void fdd(arr& xdd, const arr& x);

void MT::rk4(arr& x1,const arr& x0,
	 void (*df)(arr& xd,const arr& x),
	 double dt){
  uint n=x0.N;
  arr k1(n),k2(n),k3(n),k4(n);

  df(k1,x0);
  df(k2,x0+(double).5*dt*k1);
  df(k3,x0+(double).5*dt*k2);
  df(k4,x0+   dt*k3);

  x1 = x0;
  x1 += (dt/(double)6.)*(k1 + (double)2.*k2 + (double)2.*k3 + k4);
}

void (*global_ddf)(arr& xdd,const arr& x,const arr& v);
void (*global_sf) (arr& s,  const arr& x,const arr& v);
void rk_df(arr& xd,const arr& x){
  uint n=x.N/2;
  arr X; X.referTo(x);
  X.reshape(2,n);
  arr a;
  global_ddf(a,X[0],X[1]);
  xd.resize(x.N);
  xd.setVectorBlock(X[1],0);
  xd.setVectorBlock(a,n);
}
void rk_sf(arr& s,const arr& x){
  uint n=x.N/2;
  arr X; X.referTo(x);
  X.reshape(2,n);
  global_sf(s,X[0],X[1]);
}

void MT::rk4dd(arr& x1,arr& v1,const arr& x0,const arr& v0,
	 void (*ddf)(arr& xdd,const arr& x,const arr& v),
	 double dt){
  
  global_ddf = ddf;

  uint n=x0.N;

  arr X(2,n),Y(2*n);
  X[0]=x0;
  X[1]=v0;
  X.reshape(2*n);

  rk4(Y,X,rk_df,dt);

  Y.reshape(2,n);
  x1=Y[0];
  v1=Y[1];
}


bool MT::rk4_switch(arr& x1,arr& s1,const arr& x0,const arr& s0,
	 void (*df)(arr& xd,const arr& x),
	 void (*sf)(arr& s,const arr& x),
	 double& dt,double tol){
  uint i,sn;
  arr sa=s0,sb,sm,xa=x0,xb,xm; //states at times a,m,t
  rk4(xb,x0,df,dt);
  sf(sb,xb);
  //CHECK(sa.N==sb.N,"inconsistent state indicators");
  bool change=false;
  sn=sa.N<sb.N?sa.N:sb.N;
  for(i=0;i<sn;i++) if(s0(i)*sb(i)<0.){
    change=true;
    break;
  }
  if(!change){ x1=xb; s1=sb; return false; }//no problems: no switch

  //we have a switch - so we must find it precisely!
  double a=0.,b=dt; //time interval [a,b]
  double m,min_m;   //where to cut the interval (determined by linear interpolation)

  cout <<"entering zero-crossing detection loop" <<endl;
  for(;fabs(b-a)>tol;){
    //compute new m
    min_m=m=b;
    sn=sa.N<sb.N?sa.N:sb.N;
    for(i=0;i<sn;i++) if(sa(i)*sb(i)<0.){
      m = b - sb(i) * (b-a)/(sb(i)-sa(i));
      if(m<min_m) min_m=m;
    }
    min_m=m;
    if(m-a<.1*tol) m+=.1*tol; //really close already
    if(b-m<.1*tol) m-=.1*tol; //really close already
    rk4(xm,x0,df,m);
    sf(sm,xm);
    change=false;
    sn=s0.N<sm.N?s0.N:sm.N;
    for(i=0;i<sn;i++) if(s0(i)*sm(i)<0.){ change=true; break; }

    //cout <<"a=" <<a <<" b=" <<b <<" m=" <<m <<" sa=" <<sa <<" sb=" <<sb <<" sm=" <<sm <<endl;
    cout <<" sm=" <<sm <<endl;
    if(!change){
      a=m;
      sa=sm;
      xa=xm;
    }else{
      b=m;
      sb=sm;
      xb=xm;
    }
  }

  //take right limit of time interval
  dt=b;
  x1=xb;
  s1=sb;
  cout <<"DONE" <<endl <<"dt=" <<dt <<" s1=" <<s1 <<endl;
  return true;
}

bool MT::rk4dd_switch(arr& x1,arr& v1,arr& s1,const arr& x0,const arr& v0,const arr& s0,
	 void (*ddf)(arr& xdd,const arr& x,const arr& v),
	 void (*sf)(arr& s,const arr& x,const arr& v),
	 double& dt,double tol){
  
  global_ddf = ddf;
  global_sf  = sf;

  uint n=x0.N;

  arr X(2,n),Y(2*n);
  X[0]=x0;
  X[1]=v0;
  X.reshape(2*n);

  bool change=rk4_switch(Y,s1,X,s0,rk_df,rk_sf,dt,tol);

  Y.reshape(2,n);
  x1=Y[0];
  v1=Y[1];
  return change;
}
