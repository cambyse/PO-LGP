#include "code.h"
#include <Optim/lagrangian.h>

//==============================================================================

void runFilterWithLinearPolicy(const arr& h_opt, const arr& th_opt, uint T){
  RacerEnvironment R;
  LinearPolicy pi;
  pi.shiftOffsetInterval = 200;
  MyFilter F(h_opt({0, MyFilter::w_dim()-1}));

  mlr::Rollouts xi(R, pi, F, T, 1.);

  R.display = true;
  R.noise = .1;
  arr th = th_opt;
  xi.rollout(1, th.reshape(1,4));
  cout <<"terminalTime=" <<xi.terminalTimes.first() <<" R=" <<xi.avgReturn <<endl;
}

//==============================================================================

struct MyController : mlr::Policy {
  //controller parameters
  arr Uh, Ua, U0;

  MyController(const arr& w_pi){
    uint i=0;
    arr Ah = w_pi( {i, i+4 -1} );  Ah.reshape(1,4);  i+=4;
    arr Au = w_pi( {i, i+1 -1} );  Au.reshape(1,1);  i+=1;
    arr A0 = w_pi( {i, i+1 -1} );  A0.reshape(1);  i+=1;

    Uh = w_pi( {i, i+4 -1} );  Uh.reshape(1,4);  i+=4;
    Ua = w_pi( {i, i+1 -1} );  Ua.reshape(1,1);  i+=1;
    U0 = w_pi( {i, i+1 -1} );  U0.reshape(1);  i+=1;
    CHECK_EQ(i, w_pi.N, "");
  }

  uint getThetaDim(const mlr::Environment& E, const mlr::Filter& F) const { return 0; }
  arr getInitialization(const mlr::Environment& E, const mlr::Filter& F, bool randomized=true){ return  arr(); }
  double sampleAction(arr& action, arr& dLogPAction, const arr& inputFeatures, const arr& theta, uint t){
    arr a = {.0};
    action = Uh * inputFeatures + U0 + Ua * a;
    return 1.;
  }
};

//==============================================================================

void runFilterWithMyPolicy(uint T){
  arr x = FILE("z.x_opt");
  arr w_h = x({ 0, MyFilter::w_dim()-1 });
  arr w_pi = x({ MyFilter::w_dim(), -1 });

  RacerEnvironment R;
  MyFilter F(w_h);
  MyController C(w_pi);

  mlr::Rollouts xi(R, C, F, T, 1.);

  R.display = true;
  R.noise = .1;
  xi.rollout(1, arr(1,1, {0.}));
  cout <<"terminalTime=" <<xi.terminalTimes.first() <<" R=" <<xi.avgReturn <<endl;
}

//==============================================================================

arr getModelPolicyParameters(){
  RacerEnvironment R;
  LinearPolicy pi;

  arr theta = pi.getInitialization(R, R);

  mlr::Rollouts xi(R, pi, R, 20, 1.);

  ScalarFunction f = [&xi](arr& g, arr& H, const arr& x) -> double{
    if(&g){
      xi.rollout(30, x, 2e-2);   g = -xi.getGradient_LinearRegression();
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

//  xi.T = 100;
//  //  optRprop(theta, f, OPT(verbose=2));
//  //  optGradDescent(theta, f, OPT(verbose=2));
//  optGrad(theta, f, OPT(verbose=2));

  xi.T = 200;
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
  pi.shiftOffsetInterval = 200;

  mlr::Rollouts xi(R, pi, R, 1000, 1.);

  R.display = true;
  R.noise = .1;
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

void ReLearn::createNet(int T, uint errSteps){
  arr h = FILE("z.h");
  arr u = FILE("z.u");
  arr y = FILE("z.yorg");
  arr M = FILE("ana_model");

  uint dh = 4;
  uint dy = 4;
  uint ds = 4;

  if(T==-1) T = h.d0;

  // filter parameters
  Variable *_Hh = N.newConstant(STRING("Hh"), randn(dh,dh), true );
  Variable *_Hu = N.newConstant(STRING("Hu"), randn(dh,1), true );
  Variable *_Hy = N.newConstant(STRING("Hy"), randn(dh,dy), true );
  Variable *_Hy0= N.newConstant(STRING("Hy0"), randn(dh,dy), true );
  Variable *_H0 = N.newConstant(STRING("H0"), randn(dh), true );

//  Variable *_Hsel = N.newConstant(STRING("Hhsel"), eye(4).sub(1,-1,0,-1), false );
//  Variable *_Hsel0 = N.newConstant(STRING("Hhsel0"), zeros(dh), false );

  // toy dynamics parameters
  Variable *_Ss = N.newConstant(STRING("Ss"), M.sub(0,-2,0,-1), false ); //not subject to optimization!
//  Variable *_Su = N.newConstant(STRING("Su"), M({dim_s,dim_s}), false );
  Variable *_Sa = N.newConstant(STRING("Sa"), ~M.sub(-1,-1,0,-1), false );
  Variable *_S0 = N.newConstant(STRING("S0"), zeros(ds), false );

  // toy state parameters (phi mapping)
  Variable *_SSh = N.newConstant(STRING("SSh"), eye(ds), false ); //not subject to optimization!
  Variable *_SS0 = N.newConstant(STRING("SS0"), zeros(ds), false );

//16+4+16+16+4+4+2+4+1

  // toy action parameters
  Variable *_Ah = N.newConstant(STRING("Ah"), randn(1,dh), true ); //not subject to optimization!
  Variable *_Au = N.newConstant(STRING("Au"), randn(1,1), true );
  Variable *_A0 = N.newConstant(STRING("A0"), randn(1), true );

  // decoded control parameters
  Variable *_Uh = N.newConstant(STRING("Uh"), randn(1,dh), true ); //not subject to optimization!
//  Variable *_Ua = N.newConstant(STRING("Ua"), randn(1,1), true );
  Variable *_U0 = N.newConstant(STRING("U0"), randn(1), true );

  N.G.getRenderingInfo(_Hh->n).skip=true;
  N.G.getRenderingInfo(_Hu->n).skip=true;
  N.G.getRenderingInfo(_Hy->n).skip=true;
  N.G.getRenderingInfo(_Hy0->n).skip=true;
  N.G.getRenderingInfo(_H0->n).skip=true;
//  N.G.getRenderingInfo(_Hsel->n).skip=true;
//  N.G.getRenderingInfo(_Hsel0->n).skip=true;
  N.G.getRenderingInfo(_Ss->n).skip=true;
//  N.G.getRenderingInfo(_Su->n).skip=true;
  N.G.getRenderingInfo(_Sa->n).skip=true;
  N.G.getRenderingInfo(_S0->n).skip=true;
  N.G.getRenderingInfo(_SSh->n).skip=true;
  N.G.getRenderingInfo(_SS0->n).skip=true;
  N.G.getRenderingInfo(_Ah->n).skip=true;
  N.G.getRenderingInfo(_Au->n).skip=true;
  N.G.getRenderingInfo(_A0->n).skip=true;

  //-- initializations (should be initialized with observation_zero, which is missing!)
  Variable *_h = N.newConstant(STRING("h_0"), zeros(dh), false ); //not subject to optimization!
  Variable *_y = N.newConstant(STRING("y_0"), zeros(y.d1), false );
  Variable *_s = N.newConstant(STRING("s_0"), zeros(ds), false );

  for(uint t=0;t<(uint)(T-1);t++){
    // data constants
    Variable *_u = N.newConstant(STRING("u_"<<t), u[t], false );
    Variable *_yDash = N.newConstant(STRING("y_"<<t+1), y[t], false ); //TODO: the time indexing of y is wrong!!!
    Variable *_hDashRef = N.newConstant(STRING("href_"<<t+1), h[t+1], false );
//    Variable *_hDashRef = N.newFunction(STRING("href_"<<t+1), {_hDashOrg, _Hsel, _Hsel0}, new Linear(), TUP(dh) );
    // filter
    Variable *_hDash = N.newFunction(STRING("h_"<<t+1), {_h, _u, _yDash, _y, _Hh, _Hu, _Hy, _Hy0, _H0}, new Linear(), TUP(dh) );

    // toy action
//    Variable *_a = N.newFunction(STRING("a_"<<t), {_h, _u, _Ah, _Au, _A0}, new Linear(), TUP(1) );

    // toy state
//    Variable *_sDash = N.newFunction(STRING("s_"<<t+1), {_hDash, _SSh, _SS0}, new Linear(), TUP(ds) );
//    Variable *_sDashRef = N.newFunction(STRING("sref_"<<t+1), {_s, _a, _Ss, _Sa, _S0}, new Linear(), TUP(ds) );

    // decoded control
//    Variable *_uRef = N.newFunction(STRING("uref_"<<t), {_h, /*_a,*/ _Uh, /*_Ua,*/ _U0}, new Linear(), TUP(u.d1) );


    _y = _yDash;
    _h = _hDash;
//    _s = _sDash;

    if(!(t%errSteps)){
      Variable *err_h = N.newFunction(STRING("errh_"<<t+1), {_hDash, _hDashRef}, new Difference(1.), TUP(dh), OT_sumOfSqr);
//      Variable *err_s = N.newFunction(STRING("errs_"<<t+1), {_sDash, _sDashRef}, new Difference(1.), TUP(dim_s), OT_sumOfSqr);
//      Variable *err_u = N.newFunction(STRING("erru_"<<t), {_u, _uRef}, new Difference(1.), TUP(1), OT_sumOfSqr);
    }
  }

  layoutNet();

}

//==============================================================================

void ReLearn::checkNet(const arr& x){
  arr z = x;
  if(!z.N) z = N.getAllParameters();

  N.setAllParameters(z);
  N.fwdCompute();
//  N.reportAllParameters();
  for(uint i=0;i<10;i++){
    for(;;){ //pick a random node with vectorial value
      Variable *v = N.G.rndElem()->getValue<Variable>();
      if(v->value.nd==1){
        N.checkAllDerivatives(v);
        break;
      }
    }
  }

  checkJacobianCP(N, z, 1e-4);
}

//==============================================================================

void ReLearn::layoutNet(){
  N.G.getRenderingInfo(NULL).dotstyle <<", layout=\"neato\", splines=\"true\""; //overlap=\"false\",
  uintA yt;
  for(Node *n:N.G){
    Variable& var = n->get<Variable>();
    if(var.ot) N.G.getRenderingInfo(n).dotstyle <<", color=red";
    Constant *f = dynamic_cast<Constant*>(var.f);
    if(N.G.getRenderingInfo(n).skip  || (f && f->isParameter)){ //N.G.getRenderingInfo(n).skip){
      N.G.getRenderingInfo(n).dotstyle <<", shape=box";
      N.G.getRenderingInfo(n).skip = true;
    }else{
      mlr::String s;
      uint t;
      s.read(n->keys(0),"","_", 1);
      n->keys(0) >>t;
      if(t>=yt.N) yt.append(consts<uint>(0, t-yt.N+1));
      N.G.getRenderingInfo(n).dotstyle <<", pos=\""<<2*t<<"," <<yt(t) <<"!\"";
      yt(t)++;
    }
  }
}

//==============================================================================

void ReLearn::writeData(const arr& x){
  N.setAllParameters(x);
  N.fwdCompute();

  ofstream fil("z.data");

  //-- write header
  bool skip=true;
  for(Node *n:N.G){
    if(n->keys(0).startsWith("u_")){
      if(!skip){ fil <<endl; break; }
      skip=false;
    }
    if(!skip){
      uint d = n->get<Variable>().value.N;
      mlr::String s;
      n->keys(0).resetIstream();
      s.read(n->keys(0),"","_");
      fil <<s <<' ';
      for(uint i=0;i<d;i++) fil <<s <<'.' <<i <<' ';
    }
  }

  //-- write data
  skip=true;
  for(Node *n:N.G){
    if(n->keys(0).startsWith("u_")){
      if(!skip) fil <<endl;
      skip=false;
    }
    if(!skip){
      fil <<n->keys(0) <<' ' <<n->get<Variable>().value <<' ';
    }
  }

#if 0
  for(Node *n:N.G){
    if(n->keys(0).startsWith("u_")) fil <<endl;
    if(n->keys(0).startsWith("h_")
       || n->keys(0).startsWith("href_")
       || n->keys(0).startsWith("y_")
       || n->keys(0).startsWith("s_")
       || n->keys(0).startsWith("sref_")
       || n->keys(0).startsWith("u_")
       || n->keys(0).startsWith("uref_")
       )
      fil <<n->keys(0) <<' ' <<n->get<Variable>().value <<' ';
  }
#endif

  fil.close();
}

//==============================================================================

void ReLearn::trainModel(bool init_with_previous){
  arr x = N.getAllParameters();
  //checkJacobianCP(N, x, 1e-4);

  //-- load optimal parameters for filter (optimized before)
  if(false){
    arr h_opt = FILE("h_opt");
    uint hd = MyFilter::w_dim();
    x({ 0, hd-1}) = h_opt( {0,hd-1} );
    N.setAllParameters(x);
  }

  if(init_with_previous){
    FILE("z.x_opt") >>x; //load from previous iteration
    rndGauss(x, 1e-10, true);
    N.setAllParameters(x);
  }

//  cout <<"before training: ";
//  evaluateConstrainedProblem(x, N, cout);
//  x = N.getAllParameters();
//  FILE("z.x_opt_before") <<x;
//  FILE("z.before") <<N;

  optConstrained(x, NoArr, N, OPT(verbose=2));

  cout <<"after training: ";
  evaluateConstrainedProblem(x, N, cout);

//  N.clearAllPartialDerivatives();
  FILE("z.x_opt_ascii") <<x;
  x.write(FILE("z.x_opt"), NULL, NULL, NULL, true, true);
//  FILE("z.after") <<N;

  writeData(x);
}

//==============================================================================


