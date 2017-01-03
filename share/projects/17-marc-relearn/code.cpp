#include "code.h"

//==============================================================================

arr getModelPolicyParameters(){
  RacerEnvironment R;
  LinearPolicy pi;

  arr theta = pi.getInitialization(R, R);

  mlr::Rollouts xi(R, pi, R, 20, 1.);

  ScalarFunction f = [&xi](arr& g, arr& H, const arr& x) -> double{
    if(&g){
      xi.rollout(30, x, 1e-2);   g = -xi.getGradient_LinearRegression();
      //      xi.rollout(30, x);   g = -xi.getGradient_GPOMDP();
      //      xi.rollout(30, x);   g = -xi.getGradient_REINFORCE();
      return -xi.avgReturn;
    }
    return -xi.rollout(1, x);
  };

  xi.T = 20;
  //  optRprop(theta, f, OPT(verbose=2));
  //  optGradDescent(theta, f, OPT(verbose=2));
  optGrad(theta, f, OPT(verbose=2));

  xi.T = 50;
  //  optRprop(theta, f, OPT(verbose=2));
  //  optGradDescent(theta, f, OPT(verbose=2));
  optGrad(theta, f, OPT(verbose=2));

  xi.T = 150;
  //  optRprop(theta, f, OPT(verbose=2));
  //  optGradDescent(theta, f, OPT(verbose=2));
  optGrad(theta, f, OPT(verbose=2));


  R.display = true;
  xi.T = 200;
  xi.rollout(1, theta);

  FILE("z.thetaOpt") <<theta;

  return theta;
}

//==============================================================================

void testGradients(){
  RacerEnvironment R;
  LinearPolicy pi;

  arr theta = pi.getInitialization(R, R);

  mlr::Rollouts xi(R, pi, R, 20, .9);

  cout <<"** basic rollout:" <<endl;
  xi.rollout(1, theta);
  cout <<"terminalTime=" <<xi.terminalTimes.first() <<" R=" <<xi.avgReturn <<endl;

  uint M=50;
  arr G(10, theta.N);

  //  cout <<"** Vanilla gradient requires many samples to be accurate:" <<endl;
  //  for(uint m=0; m<G.d0; m++){
  //    xi.rollout(M, theta);
  //    G[m] = xi.getGradient_Vanilla();
  //    cout <<"gradient Vanilla = " <<G[m] <<endl;
  //  }
  //  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** NATURAL gradient" <<endl;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta);
    G[m] = xi.getFisherMatrix()*xi.getNaturalQParams();
    cout <<"gradient NATURAL = " <<G[m] <<endl;
    //    cout <<"gradient NATURAL = " <<xi.getFisherMatrix()*G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** REINFORCE gradient requires many samples to be accurate:" <<endl;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta);
    G[m] = xi.getGradient_REINFORCE();
    cout <<"gradient REINFORCE = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** GPOMDP gradient has less variance?" <<endl;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta);
    G[m] = xi.getGradient_GPOMDP();
    cout <<"gradient GPOMDP = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** LinReg gradient" <<endl;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta, 1e-1);
    G[m] = xi.getGradient_LinearRegression();
    cout <<"gradient LinReg = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** LinReg gradient without policy variance" <<endl;
  pi.exploration=.0;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta, 1e-1);
    G[m] = xi.getGradient_LinearRegression();
    cout <<"gradient LinReg = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  pi.exploration=.0;
  cout <<"** finite difference gradient converges fast (fixed rndSeed for each batch):" <<endl;
  cout <<"gradient FD = " <<xi.getGradient_FiniteDifference(theta, 10) <<endl;

  cout <<"** with fixing the rndSeed, lin reg is pretty accurate (like finite difference)" <<endl;
  xi.rollout(10, theta, 1e-3, 1);
  cout <<"gradient LinReg = " <<xi.getGradient_LinearRegression() <<endl;

}

//==============================================================================

void collectData(){
  arr theta = FILE("z.thetaOpt");
  theta.reshape(1,4);
  cout <<"Loaded theta=" <<theta <<endl;

  RacerEnvironment R;
  LinearPolicy pi;

  mlr::Rollouts xi(R, pi, R, 1000, 1.);

  R.display = true;
  R.noise = .6;
  xi.rollout(1, theta);

  arr h = xi.features[0];
  arr u = xi.actions[0];
  arr yorg = xi.observations[0];

  //change to atan observations!
  CHECK_EQ(yorg.d1, 4, "");
  arr y(yorg.d0, 3);
  for(uint t=0;t<y.d0;t++){
    y(t,0) = -atan(yorg(t,0)/yorg(t,1)); //TODO: minus???
    y(t,1) = yorg(t,2);
    y(t,2) = yorg(t,3);
  }

  FILE("z.h") <<h;
  FILE("z.u") <<u;
  FILE("z.y") <<y;
  FILE("z.yorg") <<yorg;

  //  mlr::wait();
}

//==============================================================================

void createNet(Net& N){
  arr h = FILE("z.h");
  arr u = FILE("z.u");
  arr y = FILE("z.yorg");

  uint T = h.d0;

//  T=600;

  // parameters
  arr Wh = randn(4,4); //ARR(4, 4, {0., 1, 0, 0});
  arr Wu = randn(4,1); //ARR(4, 1, {0., 1, 0, 0});
  arr Wy = randn(4,4); //ARR(4, 3, {0.,0,1, 0,0,0, 0,0,0, 0,1,0});
  arr Wy0 = randn(4,4); //ARR(4, 1, {0., 0, 1, 0});
  arr b = randn(4); //ARR(4, {0,0,0,0});
  Variable *_Wh = N.newConstant(STRING("Wh"), Wh, true );
  Variable *_Wu = N.newConstant(STRING("Wu"), Wu, true );
  Variable *_Wy = N.newConstant(STRING("Wy"), Wy, true );
  Variable *_Wy0 = N.newConstant(STRING("Wy0"), Wy0, true );
  Variable *_b = N.newConstant(STRING("b"), b, true );

  N.G.getRenderingInfo(_Wh->n).skip=true;
  N.G.getRenderingInfo(_Wu->n).skip=true;
  N.G.getRenderingInfo(_Wy->n).skip=true;
  N.G.getRenderingInfo(_Wy0->n).skip=true;
  N.G.getRenderingInfo(_b->n).skip=true;


  // filter initialization (should be initialized with observation_zero, which is missing!
  Variable *_h = N.newConstant(STRING("h_0"), zeros(h.d1), false );
  Variable *_y = N.newConstant(STRING("y_0"), zeros(y.d1), false );

  for(uint t=0;t<T;t++){
    // data constants
    Variable *_u = N.newConstant(STRING("u_"<<t), u[t], false );
    Variable *_yDash = N.newConstant(STRING("y_"<<t+1), y[t], false ); //TODO: the time indexing of y is wrong!!!
    Variable *_hDashRef = N.newConstant(STRING("href_"<<t+1), h[t], false );
    // filter variables
//    Variable *_uIDash = N.newFunction(STRING("u_"<<t+1), {_uI, _u, _Wuint, _auint}, new HighPassIntegrator(), TUP(u.d1));
//    Variable *_hIDash = N.newFunction(STRING("hI_"<<t+1), {_hI, _yDash, _Wint, _aint}, new HighPassIntegrator(), TUP(Wint.d0) );
//    Variable *_hFDash = N.newFunction(STRING("hF_"<<t+1), {_hF, _yDash, _Wfil, _afil}, new LowPassFilter(), TUP(Wfil.d0) );
//    Variable *_hDash = N.newFunction(STRING("h_"<<t+1), {_hIDash, _hFDash, _yDash, _uIDash, _WhI, _WhF, _Wy, _WuI, _b}, new Linear(), TUP(h.d1) );
    Variable *_hDash = N.newFunction(STRING("h_"<<t+1), {_h, _u, _yDash, _y, _Wh, _Wu, _Wy, _Wy0, _b}, new Linear(), TUP(h.d1) );
    _y = _yDash;
    _h = _hDash;

    if(!(t%2)){
      Variable *err = N.newFunction(STRING("err_"<<t+1), {_hDash, _hDashRef}, new Difference(), TUP(h.d1), OT_sumOfSqr);
    }

//    N.G.getRenderingInfo(_u->n).dotstyle <<"rank=" <<3*t;
//    N.G.getRenderingInfo(_yDash->n).dotstyle <<"rank=" <<3*t+1;
//    N.G.getRenderingInfo(_uIDash->n).dotstyle <<"rank=" <<3*t+1;
//    N.G.getRenderingInfo(_hDash->n).dotstyle <<"rank=" <<3*t+2;
//    N.G.getRenderingInfo(_hDashRef->n).dotstyle <<"rank=" <<3*t+2;
//    "constraint=false"
  }
}

//==============================================================================

void writeData(Net& N){
  ofstream fil("z.data");

  for(Node *n:N.G){
    if(n->keys(0).startsWith("h_")
       || n->keys(0).startsWith("href_")
       || n->keys(0).startsWith("y_")
       )
      fil <<n->keys(0) <<' ' <<n->get<Variable>().value <<' ';

    if(n->keys(0).startsWith("err"))
      fil <<n->keys(0) <<' ' <<n->get<Variable>().value <<endl;
  }

  fil.close();
}
