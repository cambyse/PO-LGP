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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define ITMAX 1000
#define EPS 1.0e-10
/* Here ITMAX is the maximum allowed number of iterations, while EPS
   is a small number to rectify the special case of converging to
   exactly zero function value. */
#define FREEALL free_vector(xi,1,n);free_vector(h,1,n);free_vector(g,1,n);
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#ifndef HALT
#  define HALT(msg) { printf("%s:%i:HALT:\n --- %s ---\n",__FILE__,__LINE__,msg); abort(); }
#endif

//================================================================================
//
// fwd declarations
//

void frprmn(double p[], int n, double ftol, int *iter, int maxIterations, double *fret,
	    double (*func)(double [],void*), void (*dfunc)(double [], double [],void*),
	    void *data);
void dlinmin(double p[], double xi[], int n, double *fret,
	     double (*func)(double [],void*), void (*dfunc)(double [], double [],void*),
	     void *data);
double df1dim(double x,void *data);
double dbrent(double ax, double bx, double cx,
	      double (*f)(double,void*), double (*df)(double,void*),
	      void *data,
	      double tol, double *xmin);
double f1dim(double x,void *data);
void mnbrak(double *ax, double *bx, double *cx, double *fa, double *fb, double *fc,
	    double (*func)(double,void*),void *data);

//================================================================================
//
// implementations
//

//#include "nrutil.h"
#define TOL 2.0e-4 //Tolerance passed to dbrent.

int ncom;  //Global variables communicate with df1dim.
double *pcom,*xicom,(*nrfunc)(double [],void*);
void (*nrdfun)(double [], double [],void*);
#include <math.h>
//#include "nrutil.h"
#define ZEPS 1.0e-10
#define MOV3(a,b,c, d,e,f) (a)=(d);(b)=(e);(c)=(f);
#include <math.h>
//#include "nrutil.h"
#define GOLD 1.618034
#define GLIMIT 100.0
#define TINY 1.0e-20
#define SHFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
/* Here GOLD is the default ratio by which successive intervals are
   magnified; GLIMIT is the maximum magnification allowed for a
   parabolic-fit step. */



double *rmVector(int i,int j){           return (double*) malloc(j*sizeof(double) ); }
void free_vector(double* p,int i,int j){ free(p); }
double MAX(double a,double b){           return a>b?a:b; }

void frprmn(double p[], int n, double ftol, int *iter, int maxIterations, double *fret,
	    double (*func)(double [],void*), void (*dfunc)(double [], double [],void*),
	    void *data)
     /*  Given a starting point p[1..n], Fletcher-Reeves-Polak-Ribiere
	 minimization is performed on a function func, using its
	 gradient as calculated by a routine dfunc. The convergence
	 tolerance on the function value is input as ftol. Returned
	 quantities are p (the location of the minimum), iter (the
	 number of iterations that were performed), and fret (the
	 minimum value of the function). The routine linmin is called
	 to perform line minimizations. */
{
  void dlinmin(double p[], double xi[], int n, double *fret,
	       double (*func)(double [],void*),void (*dfunc)(double [], double [],void*),
	       void *data);
  int j,its;
  double gg,gam,fp,dgg;
  double *g,*h,*xi;

  g=rmVector(1,n);
  h=rmVector(1,n);
  xi=rmVector(1,n);
  fp=(*func)(p,data);  //Initializations.
  (*dfunc)(p,xi,data);
  for (j=0;j<n;j++) {
    g[j] = -xi[j];
    xi[j]=h[j]=g[j];
  }
  for (its=1;its<=maxIterations;its++) { //Loop over iterations.
    *iter=its;
    dlinmin(p,xi,n,fret,func,dfunc,data); //Next statement is the normal return:
    if (2.0*fabs(*fret-fp) <= ftol*(fabs(*fret)+fabs(fp)+EPS)) {
      FREEALL;
      return;
    }
    fp= *fret;
    (*dfunc)(p,xi,data);
    dgg=gg=0.0;
    for (j=0;j<n;j++) {
      gg += g[j]*g[j];
      /* dgg += xi[j]*xi[j]; */  //This statement for Fletcher-Reeves.
      dgg += (xi[j]+g[j])*xi[j]; //This statement for Polak-Ribiere.
    }
    if (gg == 0.0) {             //Unlikely. If gradient is exactly zero then
      FREEALL;                    //we are already done.
      return;
    }
    gam=dgg/gg;
    for (j=0;j<n;j++) {
      g[j] = -xi[j];
      xi[j]=h[j]=g[j]+gam*h[j];
    }
  }
  HALT("Too many iterations in frprmn");
}

void dlinmin(double p[], double xi[], int n, double *fret,
	     double (*func)(double [],void*), void (*dfunc)(double [], double [],void*),
	     void *data)
     /* Given an n-dimensional point p[1..n] and an n-dimensional
	direction xi[1..n], moves and resets p to where the function
	func(p) takes on a minimum along the direction xi from p, and
	replaces xi by the actual vector displacement that p was
	moved. Also returns as fret the value of func at the returned
	location p. This is actually all accomplished by calling the
	routines mnbrak and dbrent. */
{
  int j;
  double xx,xmin,fx,fb,fa,bx,ax;
  ncom=n; //Define the global variables.
  pcom=rmVector(1,n);
  xicom=rmVector(1,n);
  nrfunc=func;
  nrdfun=dfunc;
  for (j=0;j<n;j++) {
    pcom[j]=p[j];
    xicom[j]=xi[j];
  }
  ax=0.0; //Initial guess for brackets.
  xx=1.0;
  mnbrak(&ax,&xx,&bx,&fa,&fx,&fb,f1dim,data);
  *fret=dbrent(ax,xx,bx,f1dim,df1dim,data,TOL,&xmin);
  for (j=0;j<n;j++) { //Construct the vector results to return.
    xi[j] *= xmin;
    p[j] += xi[j];
  }
  free_vector(xicom,1,n);
  free_vector(pcom,1,n);
}

double df1dim(double x,void *data){
  int j;
  double df1=0.0;
  double *xt,*df;
  xt=rmVector(1,ncom);
  df=rmVector(1,ncom);
  for (j=0;j<ncom;j++) xt[j]=pcom[j]+x*xicom[j];
  (*nrdfun)(xt,df,data);
  for (j=0;j<ncom;j++) df1 += df[j]*xicom[j];
  free_vector(df,1,ncom);
  free_vector(xt,1,ncom);
  return df1;
}

double dbrent(double ax, double bx, double cx,
	      double (*f)(double,void*), double (*df)(double,void*),
	      void *data,
	      double tol, double *xmin)
     /* Given a function f and its derivative function df, and given a
	bracketing triplet of abscissas ax, bx, cx [such that bx is
	between ax and cx, and f(bx) is less than both f(ax) and
	f(cx)], this routine isolates the minimum to a fractional
	precision of about tol using a modification of Brent’s method
	that uses derivatives. The abscissa of the minimum is returned
	as xmin, and the minimum function value is returned as dbrent,
	the returned function value. */
{
  int iter,ok1,ok2; //Will be used as flags for whether pro
  double a,b,d=0.0,d1,d2,du,dv,dw,dx,e=0.0; //posed steps are acceptable or not.
  double fu,fv,fw,fx,olde,tol1,tol2,u,u1,u2,v,w,x,xm;
  //Comments following will point out only differences from the routine brent. Read that
  //routine first.
  a=(ax < cx ? ax : cx);
  b=(ax > cx ? ax : cx);
  x=w=v=bx;
  fw=fv=fx=(*f)(x,data);
  dw=dv=dx=(*df)(x,data);
  for (iter=1;iter<=ITMAX;iter++) {
    xm=0.5*(a+b);
    tol1=tol*fabs(x)+ZEPS;
    tol2=2.0*tol1;
    if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
      *xmin=x;
      return fx;
    }
    if (fabs(e) > tol1) {
      d1=2.0*(b-a);// Initialize these d’s to an out-of-bracket
      d2=d1; //value.
      if (dw != dx) d1=(w-x)*dx/(dx-dw); //Secant method with one point.
      if (dv != dx) d2=(v-x)*dx/(dx-dv); //And the other.
      //Which of these two estimates of d shall we take? We will insist that they be within
      //the bracket, and on the side pointed to by the derivative at x:
      u1=x+d1;
      u2=x+d2;
      ok1 = (a-u1)*(u1-b) > 0.0 && dx*d1 <= 0.0;
      ok2 = (a-u2)*(u2-b) > 0.0 && dx*d2 <= 0.0;
      olde=e; //Movement on the step before last.
      e=d;
      if (ok1 || ok2) { //Take only an acceptable d, and if
	//   both are acceptable, then take
	//the smallest one.
	if (ok1 && ok2)
	  d=(fabs(d1) < fabs(d2) ? d1 : d2);
	else if (ok1)
	  d=d1;
	else
	  d=d2;
	if (fabs(d) <= fabs(0.5*olde)) {
	  u=x+d;
	  if (u-a < tol2 || b-u < tol2)
	    d=SIGN(tol1,xm-x);
	} else { //Bisect, not golden section.
	  d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
	  //Decide which segment by the sign of the derivative.
	}
      } else {
	d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
      }
    } else {
      d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
    }
    if (fabs(d) >= tol1) {
      u=x+d;
      fu=(*f)(u,data);
    } else {
      u=x+SIGN(tol1,d);
      fu=(*f)(u,data);
      if (fu > fx) { //If the minimum step in the downhill
	//direction takes us uphill, then
	//we are done.
	*xmin=x;
	return fx;
      }
    }
    du=(*df)(u,data); //Now all the housekeeping, sigh.
    if (fu <= fx) {
      if (u >= x) a=x; else b=x;
      MOV3(v,fv,dv, w,fw,dw);
      MOV3(w,fw,dw, x,fx,dx);
      MOV3(x,fx,dx, u,fu,du);
    } else {
      if (u < x) a=u; else b=u;
      if (fu <= fw || w == x) {
	MOV3(v,fv,dv, w,fw,dw);
	MOV3(w,fw,dw, u,fu,du);
      } else if (fu < fv || v == x || v == w) {
	MOV3(v,fv,dv, u,fu,du);
      }
    }
  }
  HALT("Too many iterations in routine dbrent");
  return 0.0; //Never get here.
}

double f1dim(double x,void *data)
     //Must accompany linmin.
{
  int j;
  double f,*xt;
  xt=rmVector(1,ncom);
  for (j=0;j<ncom;j++) xt[j]=pcom[j]+x*xicom[j];
  f=(*nrfunc)(xt,data);
  free_vector(xt,1,ncom);
  return f;
}

void mnbrak(double *ax, double *bx, double *cx, double *fa, double *fb, double *fc,
	    double (*func)(double,void*),void *data)
     /* Given a function func, and given distinct initial points ax
	and bx, this routine searches in the downhill direction
	(defined by the function as evaluated at the initial points)
	and returns new points ax, bx, cx that bracket a minimum of
	the function. Also returned are the function values at the
	three points, fa, fb, and fc. */
{
  double ulim,u,r,q,fu,dum;
  *fa=(*func)(*ax,data);
  *fb=(*func)(*bx,data);
  if (*fb > *fa) { // Switch roles of a and b so that we can go
    //downhill
    SHFT(dum,*ax,*bx,dum); //in the direction from a to b.
    SHFT(dum,*fb,*fa,dum);
  }
  *cx=(*bx)+GOLD*(*bx-*ax); //First guess for c.
  *fc=(*func)(*cx,data);
  while (*fb > *fc) { //Keep returning here until we bracket.
    r=(*bx-*ax)*(*fb-*fc);
    q=(*bx-*cx)*(*fb-*fa);
    u=(*bx)-((*bx-*cx)*q-(*bx-*ax)*r)/
      (2.0*SIGN(MAX(fabs(q-r),TINY),q-r));
    ulim=(*bx)+GLIMIT*(*cx-*bx);
    if ((*bx-u)*(u-*cx) > 0.0) { //Parabolic u is between b and c: try it.
      fu=(*func)(u,data);
      if (fu < *fc) { //Got a minimum between b and c.
	*ax=(*bx);
	*bx=u;
	*fa=(*fb);
	*fb=fu;
	return;
      } else if (fu > *fb) { //Got a minimum between between a and u.
	*cx=u;
	*fc=fu;
	return;
      }
      u=(*cx)+GOLD*(*cx-*bx); //Parabolic fit was no use. Use default mag
      fu=(*func)(u,data);// nification.
    } else if ((*cx-u)*(u-ulim) > 0.0) { //Parabolic fit is between c and its
      fu=(*func)(u,data); //allowed limit.
      if (fu < *fc) {
	SHFT(*bx,*cx,u,*cx+GOLD*(*cx-*bx));
	SHFT(*fb,*fc,fu,(*func)(u,data));
      }
    } else if ((u-ulim)*(ulim-*cx) >= 0.0) { //Limit parabolic u to maximum
      u=ulim; //allowed value.
      fu=(*func)(u,data);
    } else { //Reject parabolic u, use default magnifica
      u=(*cx)+GOLD*(*cx-*bx); //tion.
      fu=(*func)(u,data);
    }
    SHFT(*ax,*bx,*cx,u);// Eliminate oldest point and continue.
    SHFT(*fa,*fb,*fc,fu);
  }
}
