#include<MT/algos.h>
#include<MT/plot.h>
#include<MT/ors.h>


double cost(arr *grad,const arr& f,void*){
  uint t;
  double C=0.;
  //obstacle
  /*  for(t=0;t<f.d0;t++){
    if(f(t,0)<1. && f(t,1)>0.){
      (*grad)(t,0) += .01*(f(t,0)-1.);
      (*grad)(t,1) += .01*f(t,1);
    }
  }*/

  //gravity
  for(t=0;t<f.d0;t++){
    C += f(t,1)*.001;
  }
  //tension
  for(t=1;t<f.d0;t++){
    C += sumOfSqr(f[t]-f[t-1]);
  }
  //goals
  for(t=0;t<f.d0/2;t++) C += .1*sumOfSqr(f[t]-0.);
  t=0;       C += sumOfSqr(f[t]-0.);
  t=f.d0-1;  C += sumOfSqr(f[t]-1.);

  if(grad){
    (*grad).resizeAs(f);
    (*grad).setZero();
    //gravity
    for(t=0;t<f.d0;t++){
      (*grad)(t,1) += .001;
    }
    //tension
    for(t=0;t<f.d0;t++){
      if(t>0)      (*grad)[t]() += 2.*(f[t]-f[t-1]);
      if(t+1<f.d0) (*grad)[t]() -= 2.*(f[t+1]-f[t]);
    }
    //goals
    for(t=0;t<f.d0/2;t++)      (*grad)[t]() += .2*(f[t]-0.);
    t=0;       (*grad)[t]() += 2.*(f[t]-0.);
    t=f.d0-1;  (*grad)[t]() += 2.*(f[t]-1.);
  }

  return C;
}

ors::Spline *SS;
double f(arr *grad,const arr& x,void*){
  double c=cost(grad, SS->basis*x, NULL);
  if(grad){
    (*grad) = SS->basis_trans * (*grad);
  }
  return c;
}

void testBSpline(){
  uint K=5,T=100;
  ors::Spline S;
  S.setUniformNonperiodicBasis(T,K,2);
  plotOpengl();
  plotModule.drawBox=true;
  S.plotBasis();
  doubleA P(K+1,2);
  rndUniform(P,-1,1,false);
  S.points=P;
  
  arr f;
  S.evalF(f);
  plotClear();
  plotFunction(f);
  plotFunction(S.points);
  plotPoints(S.points);
  plot();

  SS = &S;

  arr dCdf,dCdx,dCdt;
  for(uint i=0;i<100;i++){
    MT::checkGradient(::cost,NULL,f,1e-5);
    MT::checkGradient(::f,NULL,S.points,1e-5);
    cost(&dCdf,f,NULL);
    //S.partial(dCdx,dCdt,dCdf,true);
    S.partial(dCdx,dCdf);
    S.points -= .3 * dCdx;
    if(i>50){
      //S.times  -= .3 * dCdt;
      //S.setBasisAndTimeGradient();
    }
    S.evalF(f);
    cout <<cost(NULL, f, NULL) <<endl;

    plotClear();
    plotFunction(f);
    plotFunction(S.points);
    plotPoints(S.points);
    plot(false);
  }

  plot(true);
}


void testOldSplines(){
  uint N=10;

  doubleA P(N,2);
  rndUniform(P,-1,1,false);

  XSpline S;
  S.referTo(P);
  S.type(false,1.); //is default

  std::ofstream os("z.spline");
  MT::IOraw=true;
  doubleA x,v,dx;
  for(double t=0.;t<N-1;t+=.01){
    S.eval(t,x,v);
    if(t+.001<N-1){ S.eval(t+.001,dx); dx=(dx-x)/.001; }//numerical derivative
    os <<t <<x <<v <<dx <<std::endl;
  }

  MT::save(P,"z.points");
  gnuplot("plot 'z.spline' us 2:3 with lines,'z.points' with points");
  gnuplot("plot 'z.spline' us 2 with lines title 'z.spline f(x)','z.spline' us 4 with lines title 'v=df/dx' lw 3,'z.spline' us 6 with lines title 'numerical v'");
  gnuplot("plot 'z.spline' us 3 with lines title 'z.spline f(x)','z.spline' us 5 with lines title 'v=df/dx' lw 3,'z.spline' us 7 with lines title 'numerical v'");
}

int main(int argn,char** argv){
  testBSpline();
  //testOldSplines();

  return 0;
}
