#include <Core/array.h>
#include <Gui/plot.h>
#include <Optim/constrained.h>
#include <Optim/optimization.h>

static double a=1e-4;
static double b0=.1;

double xi(double b, bool touch){
  //if(touch) return (a/(b+a) - 1.);
  if(touch) return -1.;
  return 0.;
}

void integrate(){
  double b=1.;
  double D=4.;
  double tau=.01;
  arr B;
  for(uint t=0;t<=D/tau;t++){
    double b_dot = xi(b, (t>200)) * b + b0;
    b = b + tau*b_dot;
    B.append(b);
  }


  plotGnuplot();
  plotFunction(B);
  plot(true);
}

//===========================================================================

struct MiniBeliefProblem:KOrderMarkovFunction {
  double tau;
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);

  uint get_T(){ return 400; }
  uint get_k(){ return 2; }
  uint dim_x(){ return 2; }
  uint dim_phi(uint t);
  uint dim_g(uint t){ return 1; }
  arr get_prefix(){ arr x(2,2); x=1.; return x; }
};

uint MiniBeliefProblem::dim_phi(uint t){
  if(t==get_T()) return 5;
  return 3;
}

void MiniBeliefProblem::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint T=get_T(), n=dim_x(), k=get_k();

  //-- pos transition costs
  double H=10.;
  phi.append(H*(x_bar(2,0)-2.*x_bar(1,0)+x_bar(0,0))); //penalize acceleration

  //-- belief transition costs
  double m=.1;
  double xi = MT::sigmoid(-x_bar(1,0)/m+1);
  double belPrec=100.;
  phi.append(belPrec*(x_bar(2,1) - (1.-tau*xi)*x_bar(1,1) - tau*b0)); //penalize acceleration

  //-- belief final costs
  if(t==T){
    phi.append(x_bar(2,0));
    phi.append(x_bar(2,1));
  }

  //-- constraint
  phi.append(-x_bar(2,0));

  CHECK(phi.N==dim_phi(t),"");

  if(&J){ //we also need to return the Jacobian
    J.resize(phi.N, k+1, n).setZero();

    //-- pos transition costs
    J(0,2,0) = H*1.;  J(0,1,0) = -H*2.;  J(0,0,0) = H*1.;

    //-- belief transition costs
    J(1,2,1) = belPrec;  J(1,1,1) = -belPrec*(1.-tau*xi);
    J(1,1,0) = belPrec * (tau*x_bar(1,1)) * xi*(1.-xi)/m*(-1.);

    //-- belief final costs
    if(t==T){
      J(2,2,0)=1.;
      J(3,2,1)=1.;
    }

    //-- constraint
    J(phi.N-1,2,0) = -1.;
  }
}

//===========================================================================

void optim(){

  MiniBeliefProblem P;
  P.tau=.01;
  uint T=P.get_T();
  uint n=P.dim_x();

  //-- gradient check
  arr x(T+1,n);
  for(uint k=0;k<2;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(P), x, 1e-4);
  }

  //-- optimize
  rndUniform(x,-10.,-1.);
  x=1.;
  optConstrained(x, Convert(P), OPT(verbose=2, useAdaptiveDamping=false, constrainedMethod=augmentedLag));
//  optNewton(x, Convert(P), OPT(verbose=2, useAdaptiveDamping=true));

  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2", true, true);
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

//  integrate();
  optim();

  return 0;
}


