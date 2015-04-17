#include <Core/array.h>
#include <Gui/plot.h>
#include <Optim/optimization.h>

//static double a=1e-4;
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
  enum BeliefRewardModel { none=0, beliefGoal, sticky } mode;
  double margin, alpha, stickyPrec, beliefDynPrec, beliefGoalPrec, posGoalPrec;

  MiniBeliefProblem(){
    margin = MT::getParameter<double>("margin", .01);
    alpha = MT::getParameter<double>("alpha", 10.);
    stickyPrec =  MT::getParameter<double>("stickyPrec", .01);
    beliefDynPrec =  MT::getParameter<double>("belDynPrec", 10.);
    beliefGoalPrec =  MT::getParameter<double>("belGoalPrec", 10.);
    posGoalPrec =  MT::getParameter<double>("posGoalPrec", 10.);
  }
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);

  uint get_T(){ return 400; }
  uint get_k(){ return 2; }
  uint dim_x(){ return 2; }
  uint dim_phi(uint t);
  uint dim_g(uint t){ return 1; } //1
  arr get_prefix(){ arr x(2,dim_x()); x=1.; return x; }
};

//===========================================================================

uint MiniBeliefProblem::dim_phi(uint t){
  if(t==get_T()){
    if(mode==none) return 4;
    return 5;
  }
  if(mode==sticky) return 4;
  return 3;
}

void MiniBeliefProblem::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint T=get_T(), n=dim_x(), k=get_k();
  phi.clear();

  //-- pos transition costs
  double H=100.;
  phi.append(H*(x_bar(0,0)-2.*x_bar(1,0)+x_bar(2,0))); //penalize acceleration

  //-- belief transition costs
  double xi = MT::sigmoid(-x_bar(1,0)/margin+1);
  phi.append(beliefDynPrec*(x_bar(2,1) - (1.-tau*alpha*xi)*x_bar(1,1) - tau*b0)); //penalize acceleration
//  phi.append(0.);

  //-- final pos cost
  if(t==T) phi.append(posGoalPrec*(x_bar(2,0)-.1));

  //-- belief final costs
  if(t==T && mode==beliefGoal) phi.append(beliefGoalPrec*x_bar(2,1));

  //-- sticky reward
  if(mode==sticky) phi.append(stickyPrec*(x_bar(2,0)+.2));

  //-- constraint
  phi.append(-x_bar(2,0));

  CHECK_EQ(phi.N,dim_phi(t),"");

  if(&J){ //we also need to return the Jacobian
    J.resize(phi.N, k+1, n).setZero();

    //-- pos transition costs
    J(0,0,0) = H*1.;  J(0,1,0) = -H*2.;  J(0,2,0) = H*1.;

    //-- belief transition costs
    J(1,2,1) = beliefDynPrec;  J(1,1,1) = -beliefDynPrec*(1.-tau*alpha*xi);
    J(1,1,0) = beliefDynPrec * (tau*x_bar(1,1)) * alpha*xi*(1.-xi)/margin*(-1.);

    //-- final pos cost
    if(t==T) J(2,2,0) = posGoalPrec;

    //-- belief final costs
    if(t==T && mode==beliefGoal) J(3,2,1) = beliefGoalPrec;

    //-- sticky reward
    if(mode==sticky) J(phi.N-2,2,0) = stickyPrec;

    //-- constraint
    J(phi.N-1,2,0) = -1.;
  }
}

//===========================================================================

void optim(){

  MiniBeliefProblem P;
  P.mode = (MiniBeliefProblem::BeliefRewardModel)MT::getParameter<int>("mode",1);
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
  rndUniform(x,-1.,1.);
  x=1.;
  arr dual;
  if(P.dim_g(0)>0){
    optConstrained(x, dual, Convert(P));
  }else{
    optNewton(x, Convert(P), OPT(verbose=2, stopTolerance=1e-3, constrainedMethod=augmentedLag));
  }

  for(double& x:dual) if(x>0.) x=1.;

  write(LIST<arr>(x, dual),"z.output");
  gnuplot("plot [:][-.1:] 'z.output' us 1 t 'position','' us 2 t 'uncertainty', '' us 3 t 'dual>0'", true, true);
//  write(LIST<arr>(x, dual),"z.output");
//  gnuplot("plot [:][-.1:] 'z.output' us 1 t 'position','' us 2 t 'uncertainty', '' us 3 t 'dual>0'", true, true);
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

//  integrate();
  optim();

  return 0;
}


