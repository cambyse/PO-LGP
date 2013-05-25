#include <Optim/optimization.h>
#include <Optim/optimization_benchmarks.h>
#include <stdlib.h>

void displayFunction(ScalarFunction& F){
  arr X, Y;
  X.setGrid(2,-1.,1.,100);
  Y.resize(X.d0);
  for(uint i=0;i<X.d0;i++) Y(i) = F.fs(NoArr, NoArr, X[i]);
  Y.reshape(101,101);
  write(LIST<arr>(Y),"z.fct");
  gnuplot("splot [-1:1][-1:1] 'z.fct' matrix us ($1/50-1):($2/50-1):3 w l", false, true);
}

void testGradDescent(ScalarFunction& F){
  uint d=MT::getParameter<uint>("dim", 2);
  arr x(d),x0;
  rnd.clockSeed();
  for(uint k=0;k<3;k++){
    rndUniform(x, -1., 1.);
    x0=x;
    cout <<"x0=" <<x0 <<endl;
    checkGradient(F, x, 1e-4);

    optGradDescent(x, F, OPT2(verbose=2, stopTolerance=1e-3));
    cout <<"x_opt=" <<x <<endl;
    gnuplot("load 'plt'", false, true);
    MT::wait();

    x=x0;
    optRprop(x, F, OPT2(verbose=2, stopTolerance=1e-3));
    cout <<"x_opt=" <<x <<endl;
    gnuplot("load 'plt'", false, true);
    MT::wait();
  }
}

void testConstraint(VectorFunction& F, bool augmented){

  struct PenalizedFunction:ScalarFunction{
    VectorFunction &F;
    double mu;
    arr lambda;
    PenalizedFunction(VectorFunction &_F):F(_F){ mu=10.; }
    virtual double fs(arr& g, arr& H, const arr& x){
      arr phi, J;
      F.fv(phi, (&g?J:NoArr), x);

      if(!lambda.N){ lambda.resize(phi.N); lambda.setZero(); }

      double f = phi(0); //costs
      for(uint i=1;i<phi.N;i++) if(phi(i)>0. || lambda(i)>0.) f += mu * MT::sqr(phi(i));  //penalty
      for(uint i=1;i<phi.N;i++) if(lambda(i)>0.) f += lambda(i) * phi(i);  //augments

      if(&g){
        g = J[0]; //costs
        for(uint i=1;i<phi.N;i++) if(phi(i)>0. || lambda(i)>0.) g += mu * 2.*phi(i)*J[i];  //penalty
        for(uint i=1;i<phi.N;i++) if(lambda(i)>0.) g += lambda(i) * J[i];  //augments
        g.reshape(x.N);
      }

      if(&H) NIY;

      return f;
    }
    void updateLambda(const arr& x){
      arr phi;
      F.fv(phi, NoArr, x);
      for(uint i=1;i<phi.N;i++) if(phi(i)>0. || lambda(i)>0.) lambda(i) += mu * 2.*phi(i);

      for(uint i=1;i<phi.N;i++) if(lambda(i)<0.) lambda(i)=0.;

      cout <<"Update Lambda: phi=" <<phi <<" lambda=" <<lambda <<endl;
    }
  } myPenalizedFunction(F);

//  testGradDescent(myPenalizedFunction);

  uint d=MT::getParameter<uint>("dim", 2);
  arr x(d);
  rndUniform(x, -1., 1.);
  cout <<"x0=" <<x <<endl;
  checkGradient(myPenalizedFunction, x, 1e-4);

  system("rm z.grad_all");
  for(uint k=0;k<10;k++){
    optRprop(x, myPenalizedFunction, OPT3(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    //optGradDescent(x, myPenalizedFunction, OPT3(verbose=2, stopTolerance=1e-3, initStep=1e-1));

    if(augmented)  myPenalizedFunction.updateLambda(x);
    else myPenalizedFunction.mu *= 10;

    system("cat z.grad >>z.grad_all");
    cout <<"x_opt=" <<x <<" mu=" <<myPenalizedFunction.mu <<" lambda=" <<myPenalizedFunction.lambda <<endl;
  }

  system("mv z.grad_all z.grad");
  gnuplot("load 'plt'", false, true);
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  int mode = MT::Parameter<int>("mode", 0);

  switch(mode){
  case 0:{
    ChoiceFunction F;
    displayFunction(F);
    MT::wait();
    testGradDescent(F);
  } break;
  case 1:{
    ChoiceConstraintFunction F;
    testConstraint(F, false);
  } break;
  case 2:{
    ChoiceConstraintFunction F;
    testConstraint(F, true);
  } break;
  }

  return 0;
}
