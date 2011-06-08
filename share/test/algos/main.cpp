#include<MT/util.h>
#include<MT/algos.h>
#include<MT/functions.h>
#include<MT/dynamics.h>
#include<MT/plot.h>

using namespace std;

//----------------------------------------------------
// tests

void testMonSolve(){
  MonSolver s;
  double x,e=1.;
  s.init(x);
  while(e*e>1e-6){
    e=sin(x)-.5;
    s.solve(x,e);
    cout <<x <<' ' <<sin(x) <<endl;
  }
  s.init(x); e=1.;
  while(e*e>1e-6){
    e=sin(x)-(-.5);
    s.solve(x,e);
    cout <<x <<' ' <<sin(x) <<endl;
  }
}

void testSVD(){
  doubleA a,u,v,w,W;
  a.resize(7,10);
  rndGauss(a,1.,false);
  
  cout <<"matrix rank: " <<svd(a,u,w,v,true) <<endl;
  W.setDiag(w);
  cout <<norm(a - u * W * ~v) <<endl;
}

void testDeterminant(){
  //double A[4]={1.,2.,-2.,3.};
  double B[9]={1,1,2,1,1,0,0,-2,3};
  //doubleA a; a.copy(A,4); a.resizeCopy(2,2);
  doubleA a; a.setCarray(B,9); a.reshape(3,3);
  cout <<a <<"det=" <<determinant(a) <<endl;
  cout <<"co00=" <<cofactor(a,0,0) <<endl;
  cout <<"co10=" <<cofactor(a,1,0) <<endl;
}

void testRprop(){
  uint t;
  
  doubleA x(2); x(0)=10.; x(1)=9.;
  doubleA grad(2);
  Rprop gd;
  std::ofstream fout("z");
  
  for(t=0;t<1000;t++){
    grad=x;
    gd.step(x,grad);
    fout <<x.ioraw() <<std::endl;  
  }
  gnuplot("plot [0:20] 'z' us 1,'z' us 2");
}

double df1(doubleA& x,doubleA& dx){
  static doubleA a,A;
  if(!A.N){ A.setId(2); rndGauss(A,.3,true); a.resize(2); a=0.; }
  return dNNinv(x,a,A,dx);
}
void testMaximize(){
  doubleA dx,x(2); x=10.; rndGauss(x,1.,true);
  Rprop rp;
  for(uint t=0;t<100;t++){
    df1(x,dx);
    rp.step(x,dx);
  }
}

void testSymIndex(){
  TupleIndex I;
  I.init(8,12);
  
  std::cout <<I;
  I.checkValid();
}


extern void glDrawRect(float x,float y,float z,float rad);

#ifdef MT_GL
class Phase{
public:
  double x,v,a;
  void random(){ x=rnd.uni(-1.,1.); v=rnd.uni(-1,1.); a=rnd.uni(-1.,1.); }
  void step(double time=.1){
    x+=time*v;
    v+=time*a;
  }
  static void staticDraw(void* classP){ ((Phase*)classP)->glDraw(); }
  void glDraw(){
    glColor3f(.0,.0,.0);
    glDrawRect(x,v,.0,.05);
  }
};

Phase p3;

void plan(){
  uint t;
  Phase p,p1;
  p.random(); p1.random();
  OpenGL gl; gl.add(p); gl.add(p1); gl.add(p3);
  p.x=p1.x=.0;
  //double dx,dv;
  for(t=0;t<100000;t++){
    p.a=MT::phaseAccel(p.x,p.v,p1.x,p1.v,1.);
    p.step();
    //MT::phaseTangent(p.x,p.v,p1.x,p1.v,dx,dv);
    std::cout <<MT::phaseDist(p.x,p.v,p1.x,p1.v)/MT_PI*180. <<std::endl;
    //std::cout <<p.x <<' ' <<p.v <<' ' <<std::endl;
    gl.watch();
    if(fabs(p.x-p1.x)+fabs(p.v-p1.v)<.1){
      p.random(); p1.random();
    }
  }
}
#endif

void testExp(){
  doubleA X,Y;
  X.setGrid(1,-1.,1.,10*1280);
  Y.resize(X.N);
  for(uint i=0;i<X.N;i++) Y(i)=MT::approxExp(X(i,0));
  plotFunction(X,Y);
  plot();
}


static arr F;
void f(arr& y,const arr& x,void*){ y=F*x; }
void df(arr& J,const arr& x,void*){ J=F; }
void testCheckGradient(){
  F.resize(4,3);
  rndUniform(F,0.,1.,false);
  arr x(3);
  rndUniform(x,0.,1.,false);
  MT::checkGradient(f,df,NULL,x,1e-5);
}


void testFilter(){
  uint i,T=1000;
  arr x(T),y,z(T);
  for(i=0;i<T;i++) x(i) = sin(MT_2PI*i*(1./(1.+.01*i)));
  MT::bandpassFilter(y,x,10,50);
  MT::bandpassEnergy(z,x,10,50);
  plotFunction(x);
  plotFunction(y);
  plotFunction(z);
  plot();
}

#define DIM 20

namespace test{
  arr A;
  void init(uint n){
    if(A.d0!=n){ A.setId(n); rndGauss(A,.3,true); A = ~A*A; }
  }
  double f(const doubleA& x,void*){
    init(x.N);
    return (~x*(A*x))(0);
  }
  void df(doubleA& dx,const doubleA& x,void*){
    init(x.N);
    dx = 2.*A*x;
  }
}

void testRK_ddf(arr& xdd,const arr& x,const arr& v){
  if(x(0)>0.){ //free spring
    xdd = -.1*x; //- .01*v;
  }else{       //strong, damped elastic ground
    xdd = -100.*x; // - .1*v;
  }
}
void testRK(){
  double dt=.1; //dt=.1 looses energy, dt=.001 is ok
  uint D=1,T=(uint)(30./dt),t;
  arr x(T,D),v(T,D);
  x[0]=1.;
  v[0]=0.;

  for(t=1;t<T;t++){
    MT::rk4dd(x[t](),v[t](),x[t-1],v[t-1],testRK_ddf,dt);
  }
  plotFunction(x);
  plotFunction(v);
  plot();
}

void testRKswitch_ddf(arr& xdd,const arr& x,const arr& v){
  if(x(0)>0.){ //free spring
    xdd = -.1*x; //- .01*v;
  }else{       //strong, damped elastic ground
    xdd = -100.*x; // - .1*v;
  }
}
void testRKswitch_sf(arr& s,const arr& x,const arr& v){
  s=x;
}
void testRKswitch(){
  double dt=.01; //dt=.1 looses energy, dt=.001 is ok
  uint D=1,T=(uint)(30./dt),t;
  arr x(T,D),v(T,D),s(T,1);
  x[0]=1.;
  v[0]=0.;
  s[0]=1.;

  for(t=1;t<T;t++){
    MT::rk4dd_switch(x[t](),v[t](),s[t](),x[t-1],v[t-1],s[t-1],
      testRKswitch_ddf,testRKswitch_sf,dt,1e-4);
    cout <<t <<": stepsize " <<dt <<endl;
    dt=.01;
  }
  plotFunction(x);
  plotFunction(v);
  plot();
}


void testLUdecomposition(){
  arr A(6,6),L,U;
  rndGauss(A,1.,false);

  lapack_LU(L, U, A);
  cout <<A <<endl
    <<L <<endl
    <<U <<endl
    <<L*U <<endl;

}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  rnd.seed(1);
  
  testMonSolve();
  testSVD();
  testDeterminant();
  testRprop();
  testMaximize();
  testSymIndex();
  testExp();
  testCheckGradient();
  testFilter();
  testRK();
  testRKswitch();
  testLUdecomposition();

  //plan();
  
  return 0;
}
