#include <Algo/spline.h>
#include <Gui/plot.h>
#include <Ors/ors.h>

struct Cost:ScalarFunction{
  double fs(arr &g, arr &H, const arr &x){
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
    for(t=0;t<x.d0;t++){
      C += x(t,1)*.001;
    }
    //tension
    for(t=1;t<x.d0;t++){
      C += sumOfSqr(x[t]-x[t-1]);
    }
    //goals
    for(t=0;t<x.d0/2;t++) C += .1*sumOfSqr(x[t]-0.);
    t=0;       C += sumOfSqr(x[t]-0.);
    t=x.d0-1;  C += sumOfSqr(x[t]-1.);

    if(&g){
      g.resizeAs(x);
      g.setZero();
      //gravity
      for(t=0;t<x.d0;t++){
        g(t,1) += .001;
      }
      //tension
      for(t=0;t<x.d0;t++){
        if(t>0)      g[t]() += 2.*(x[t]-x[t-1]);
        if(t+1<x.d0) g[t]() -= 2.*(x[t+1]-x[t]);
      }
      //goals
      for(t=0;t<x.d0/2;t++)      g[t]() += .2*(x[t]-0.);
      t=0;       g[t]() += 2.*(x[t]-0.);
      t=x.d0-1;  g[t]() += 2.*(x[t]-1.);
    }

    return C;
  }
};

struct SplineCost:ScalarFunction{
  MT::Spline *SS;
  Cost cost;
  double fs(arr &g, arr &H, const arr &x){
    double c=cost.fs(g, NoArr, SS->basis*x);
    if(&g){
      g = SS->basis_trans*g;
    }
    return c;
  }
};

void TEST(BSpline){
  uint K=5,T=100; //6 spline point, path of length T=100

  arr X(K+1,2); //spline points
  rndUniform(X,-1,1,false);

  MT::Spline S(T, X);

  plotOpengl();
  plotModule.drawBox=true;
  S.plotBasis();
  
  arr path = S.eval();
  plotClear();
  plotFunction(path);
  plotFunction(S.points);
  plotPoints(S.points);
  plot();

  ofstream fil("z.test");
  for(uint t=0;t<=1000;t++){
    fil <<(double)t/1000 <<' ' <<S.eval(t/10) <<' ' <<S.eval((double)t/1000) <<endl;
  }
  fil.close();
  gnuplot("plot 'z.test' us 1:3, '' us 1:7",true);

  Cost cost;
  SplineCost splineCost;
  splineCost.SS = &S;

  arr grad_path, grad_X;
  for(uint i=0;i<100;i++){
    checkGradient(cost, path, 1e-5);
    checkGradient(splineCost, S.points, 1e-5);
    cost.fs(grad_path, NoArr, path);
    //S.partial(dCdx,dCdt,dCdf,true);
    S.partial(grad_X, grad_path);
    S.points -= .3 * grad_X;
    if(i>50){
      //S.times  -= .3 * dCdt;
      //S.setBasisAndTimeGradient();
    }
    path = S.eval();
    cout <<cost.fs(NoArr, NoArr, path) <<endl;

    plotClear();
    plotFunction(path);
    plotFunction(S.points);
    plotPoints(S.points);
    plot(false);
  }

  plot(true);
}


void testPath(){
  arr X(11,1);
  rndUniform(X,-1,1,false);

//  struct
  MT::Path P(X);
  cout <<"times = " <<P.times
      <<"\npoints= " <<P.points <<endl;

  MT::arrayBrackets="  ";
  FILE("z.points") <<X;

  ofstream fil("z.test");
  for(uint t=0;t<=1000;t++){
    double time=(double)t/1000;
    fil <<time <<' ' <<P.getPosition(time) <<' ' <<P.getVelocity(time) <<endl;
  }
  fil.close();
  gnuplot("plot 'z.test' us 1:2, '' us 1:3, 'z.points' us ($0/10):1 w p", true);

}

int MAIN(int argc,char** argv){
//  testBSpline();
  testPath();

  return 0;
}
