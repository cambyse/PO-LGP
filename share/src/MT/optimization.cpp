#include "optimization.h"

#ifndef CHECK_EPS
#  define CHECK_EPS 1e-8
#endif


double evaluateSP(SqrPotential& S, const arr& x){
  return scalarProduct(x,S.A*x) - 2.*scalarProduct(S.a,x) + S.c;
}

double evaluateSF(ScalarFunction& f, const arr& x){
  return f.fs(NULL, x);
}

double evaluateVF(VectorFunction& f, const arr& x){
  arr y;
  f.fv(y, NULL, x);
  return sumOfSqr(y);
}

double evaluateQCF(SqrChainFunction& f, const arr& x){
  double cost=0.;
  uint T=x.d0-1;
  cost += f.fqi(NULL, 0, x[0]);
  for(uint t=1;t<=T;t++){
    cost += f.fqi (NULL, t, x[t]);
    cost += f.fqij(NULL, t-1, t, x[t-1], x[t]);
  }
  return cost;
}

double evaluateVCF(VectorChainFunction& f, const arr& x){
  double cost=0.;
  uint T=x.d0-1;
  arr y;
  f.fvi(y, NULL, 0, x[0]);  cost += sumOfSqr(y); 
  for(uint t=1;t<=T;t++){
    f.fvi (y, NULL, t, x[t]);  cost += sumOfSqr(y); 
    f.fvij(y, NULL, NULL, t-1, t, x[t-1], x[t]);  cost += sumOfSqr(y); 
  }
  return cost;
}


conv_VectorChainFunction::conv_VectorChainFunction(VectorChainFunction& _f){
  f=&_f;
  T = f->T; //this is the T of the SqrChainFunction!
}

double conv_VectorChainFunction::fs(arr* grad, const arr& x){
  arr z;  z.referTo(x);
  z.reshape(T+1,z.N/(T+1)); //x as chain representation (splitted in nodes assuming each has same dimensionality!)
  
  double cost=0.;
  arr y,J,Ji,Jj;
  if(grad){
    grad->resizeAs(x);
    grad->setZero();
  }
  for(uint t=0;t<=T;t++){ //node potentials
    f->fvi(y, (grad?&J:NULL), t, z[t]);
    cost += sumOfSqr(y);
    if(grad){
      (*grad)[t]() += 2.*(~y)*J;
    }
  }
  for(uint t=0;t<T;t++){
    f->fvij(y, (grad?&Ji:NULL), (grad?&Jj:NULL), t, t+1, z[t], z[t+1]);
    cost += sumOfSqr(y);
    if(grad){
      (*grad)[t]()   += 2.*(~y)*Ji;
      (*grad)[t+1]() += 2.*(~y)*Jj;
    }
  }
  return cost;
}

void conv_VectorChainFunction::fv(arr& y, arr* J, const arr& x){
  arr z;  z.referTo(x);
  z.reshape(T+1,z.N/(T+1)); //x as chain representation (splitted in nodes assuming each has same dimensionality!)

  //probing dimensionality (ugly..)
  arr tmp;
  f->fvi(tmp, NULL, 0, z[0]);
  uint di=tmp.N; //dimensionality at nodes
  if(T>0) f->fvij(tmp, NULL, NULL, 0, 1, z[0], z[1]);
  uint dij=tmp.N; //dimensionality at pairs
  
  //resizing things:
  arr yi (T+1,di); //the part of y which will collect all node potentials
  arr yij(T  ,dij); //the part of y which will collect all pair potentials
  arr Ji;  Ji .resize(TUP(T+1, di, z.d0, z.d1)); //first indices as yi, last: gradient w.r.t. x
  arr Jij; Jij.resize(TUP(T  , dij, z.d0, z.d1)); //first indices as yi, last: gradient w.r.t. x
  Ji.setZero();
  Jij.setZero();
  
  arr y_loc,J_loc,Ji_loc,Jj_loc;
  uint t,i,j;
  //first collect all node potentials
  for(t=0;t<=T;t++){
    f->fvi(y_loc, (J?&J_loc:NULL), t, z[t]);
    yi[t] = y_loc;
    if(J){
      for(i=0;i<di;i++) for(j=0;j<z.d1;j++) //copy into the right place...
        Ji(TUP(t,i,t,j)) = J_loc(i,j);
    }
  }
  //first collect all pair potentials
  for(t=0;t<T;t++){
    f->fvij(y_loc, (J?&Ji_loc:NULL), (J?&Jj_loc:NULL), t, t+1, z[t], z[t+1]);
    yij[t] = y_loc;
    if(J){
      for(i=0;i<dij;i++) for(j=0;j<z.d1;j++) //copy into the right place...
        Jij(TUP(t,i,t  ,j)) = Ji_loc(i,j);
      for(i=0;i<dij;i++) for(j=0;j<z.d1;j++) //copy into the right place...
        Jij(TUP(t,i,t+1,j)) = Jj_loc(i,j);
    }
  }
  yi.reshape((T+1)*di);
  Ji.reshape((T+1)*di, x.N);
  yij.reshape(T*dij);
  Jij.reshape(T*dij, x.N);
  y=yi;  y.append(yij);
  if(J){ *J=Ji;  J->append(Jij); }
}

double conv_VectorChainFunction::fqi (SqrPotential *S, uint i, const arr& x_i){
  arr y,J;
  f->fvi(y, (S?&J:NULL), i, x_i);
  if(S){
    S->A=~J * J;
    S->a=~J * (J*x_i - y);
    S->c=sumOfSqr(J*x_i - y);
  }
  return sumOfSqr(y);
}

double conv_VectorChainFunction::fqij(PairSqrPotential *S, uint i, uint j, const arr& x_i, const arr& x_j){
  arr y,Ji,Jj;
  f->fvij(y, (S?&Ji:NULL), (S?&Jj:NULL), i, j, x_i, x_j);
  if(S){
    S->A=~Ji*Ji;
    S->B=~Jj*Jj;
    S->C=~Ji*Jj;
    S->a=~Ji*(Ji*x_i + Jj*x_j - y);
    S->b=~Jj*(Ji*x_i + Jj*x_j - y);
    S->c=sumOfSqr(Ji*x_i + Jj*x_j - y);
  }
  return sumOfSqr(y);
}




void init(SqrPotential &V, uint n){ V.A.resize(n,n); V.a.resize(n); V.A.setZero(); V.a.setZero(); V.c=0.; }

uint optNodewise(arr& x, VectorChainFunction& f, double *fmin_return, double stoppingTolerance, uint maxIter, double maxStepSize, uint verbose){

  struct MyVectorFunction:VectorFunction {
    VectorChainFunction *f;
    uint t;
    arr x_ref;
    void fv(arr& y, arr* J, const arr& x){
      arr yij,Ji,Jj;
      f->fvi (y, J, t, x);
      if(t>0){
        f->fvij(yij, (J?&Ji:NULL),  (J?&Jj:NULL), t-1, t, x_ref[t-1], x);
        y.append(yij);
        if(J) J->append(Jj);
      }
      if(t<f->T){
        f->fvij(yij, (J?&Ji:NULL),  (J?&Jj:NULL), t, t+1, x, x_ref[t+1]);
        y.append(yij);
        if(J) J->append(Ji);
      }
    }
  };

  MyVectorFunction loc_f;
  loc_f.f = &f;
  
  ofstream fil;
  if(verbose>0) fil.open("z.nodewise");
  if(verbose>0) fil <<0 <<' ' <<evaluateVCF(f, x) <<endl;
  if(verbose>1) cout <<"optNodewise initial cost " <<evaluateVCF(f, x) <<endl;

  uint k;
  for(k=0;k<maxIter;k++){
    arr x_old=x;
    for(uint t=0;t<=f.T;t++){
      loc_f.x_ref = x;
      loc_f.t = t;
      //checkGradient(loc_f, x[t], 1e-4);
      optGaussNewton(x[t](), loc_f, NULL, stoppingTolerance, 10, maxStepSize, 0);
      if(verbose>1) cout <<"optNodewise " <<k <<" > " <<t <<' ' <<evaluateVCF(f, x) <<endl;
    }
    for(uint t=f.T-1;t>0;t--){
      loc_f.x_ref = x;
      loc_f.t=t;
      //checkGradient(loc_f, x[t], 1e-4);
      optGaussNewton(x[t](), loc_f, NULL, stoppingTolerance, 10, maxStepSize, 0);
      if(verbose>1) cout <<"optNodewise " <<k <<" < " <<t <<' ' <<evaluateVCF(f, x) <<endl;
    }
    if(verbose>0) fil <<k+1 <<' ' <<evaluateVCF(f, x) <<endl;
    if(maxDiff(x,x_old)<stoppingTolerance) break;
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.nodewise' us 1:2 w l",NULL,true);
  return k;
}

uint optDynamicProgramming(arr& x, SqrChainFunction& f, double *fmin_return, double stoppingTolerance, uint maxIter, double maxStepSize, uint verbose){

  uint T=x.d0-1,n=x.d1;
  uint evals=0;
  arr y(x);
  double damping=100.;
  
  MT::Array<SqrPotential> V(T+1);
  arr Bbarinv(T,n,n);
  arr fijC(T,n,n);
  arr bbar(T,n);
  arr Id = eye(n,n);
  
  double fx = evaluateQCF(f, x);
  cout <<fx <<endl;

  ofstream fil;
  if(verbose>0) fil.open("z.DP");
  if(verbose>0) fil <<0 <<' ' <<fx <<' ' <<damping <<endl;
  if(verbose>1) cout <<"optDP initial cost " <<fx <<endl;
  
  for(uint k=0;k<maxIter;k++){
    SqrPotential fj;
    PairSqrPotential fij;
    
    //backward
    arr Bbar,C_Bbarinv;
    double cbar;
    init(V(T),n);
    for(uint t=T;t--;){
      f.fqi (&fj , t+1, x[t+1]);  evals++;
      f.fqij(&fij, t, t+1, x[t], x[t+1]);
      Bbar    = fij.B + fj.A + V(t+1).A + damping*Id;
      bbar[t] = fij.b + fj.a + V(t+1).a + damping*x[t+1];
      cbar    = fij.c + fj.c + V(t+1).c + damping*sumOfSqr(x[t+1]);
      fijC[t] = fij.C; //need to save this!
      inverse_SymPosDef(Bbarinv[t](), Bbar);
      V(t).c = cbar - scalarProduct(bbar[t], Bbarinv[t] * bbar[t]);
      C_Bbarinv  = fij.C*Bbarinv[t];
      V(t).a = fij.a - C_Bbarinv * bbar[t];
      V(t).A = fij.A - C_Bbarinv * ~fij.C;
    }
  
    //forward
    arr step;
    arr Bbarinv0,bbar0;
    bool cutSteps=false;
    f.fqi (&fj , 0, x[0]);  evals++;
    Bbar  = fj.A + V(0).A + damping*Id;
    bbar0 = fj.a + V(0).a + damping*x[0];
    cbar  = fj.c + V(0).c + damping*sumOfSqr(x[0]);
    inverse_SymPosDef(Bbarinv0, Bbar);
    step = Bbarinv0*bbar0 - y[0];
    if(maxStepSize>0. && norm(step)>maxStepSize){ cutSteps=true;  step *= maxStepSize/norm(step); }
    y[0]() += step;
    y[0] = Bbarinv0*bbar0;
    double fy = cbar - scalarProduct(bbar0, Bbarinv0 * bbar0);
    for(uint t=0;t<T;t++){
    step = Bbarinv[t]*(bbar[t] - (~fijC[t])*y[t]) - y[t+1];
      if(maxStepSize>0. && norm(step)>maxStepSize){ cutSteps=true;  step *= maxStepSize/norm(step); }
      y[t+1]() += step;
      y[t+1] = Bbarinv[t]*(bbar[t] - (~fijC[t])*y[t]);
    }
    
    //damping cost:
    double damping_cost=damping*sqrDistance(y,x);
    fy -= damping_cost;
    if(cutSteps) fy=evaluateQCF(f, y);
    //CHECK(fabs(fy-evaluateQCF(f, y))<1e-6,"");
    
    if(fy<=fx){
      if(maxDiff(x,y)<stoppingTolerance){ x=y;  fx=fy;  break; }
      x=y;
      fx=fy;
      damping /= 5.;
    }else{
      damping *= 10.;
    }

    if(verbose>1) cout <<"optDP " <<evals <<' ' <<fx <<' ' <<damping <<endl;
    if(verbose>0) fil <<evals <<' ' <<fx <<endl;
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.DP' us 1:2 w l",NULL,true);
  return evals;
}


/*double ScalarGraphFunction::f_total(const arr& X){
  uint n=X.d0;
  uintA E=edges();
  double f=0.;
  for(uint i=0;i<n;i++)    f += fi(NULL, i, X[i]);
  for(uint i=0;i<E.d0;i++) f += fij(NULL, NULL, E(i,0), E(i,1), X[E(i,0)], X[E(i,1)]);
  return f;
}

struct Tmp:public ScalarFunction{
  ScalarGraphFunction *sgf;

  double fs(arr* grad, const arr& X){
    uint n=X.d0,i,j,k;
    uintA E=sgf->edges();
    double f=0.;
    arr gi, gj;
    if(grad) (*grad).resizeAs(X);
    for(i=0;i<n;i++){
      f += sgf->fi((grad?&gi:NULL), i, X[i]);
      if(grad) (*grad)[i]() += gi;
    }
    for(k=0;k<E.d0;k++){
      i=E(k,0);
      j=E(i,1);
      f += sgf->fij((grad?&gi:NULL), (grad?&gj:NULL), i, j, X[i], X[j]);
      if(grad) (*grad)[i]() += gi;
      if(grad) (*grad)[j]() += gj;
    }
    return f;
  }
};

Tmp convert_ScalarFunction(ScalarGraphFunction& f){
  Tmp tmp;
  tmp.sgf=&f;
  return tmp;
}*/


void checkGradient(ScalarFunction &f,
                   const arr& x, double tolerance){
  arr J, dx, JJ;
  double y, dy;
  y=f.fs(&J, x);
  
  JJ.resize(x.N);
  double eps=CHECK_EPS;
  uint i;
  for(i=0; i<x.N; i++){
    dx=x;
    dx.elem(i) += eps;
    dy = f.fs(NULL, dx);
    dy = (dy-y)/eps;
    JJ(i)=dy;
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, 0);
//   MT::save(J, "z.J");
//   MT::save(JJ, "z.JJ");
  if(md>tolerance){
    MT_MSG("checkGradient (scalar) -- FAILURE -- max diff=" <<md <<" (stored in files z.J_*)");
    MT::save(J, "z.J_analytical");
    MT::save(JJ, "z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    HALT("");
  }else{
    cout <<"checkGradient (scalar) -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
}

void checkGradient(VectorFunction &f,
                   const arr& x, double tolerance){
  arr y, J, dx, dy, JJ;
  f.fv(y, &J, x);
  
  JJ.resize(y.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++){
    dx=x;
    dx.elem(i) += eps;
    f.fv(dy, NULL, dx);
    dy = (dy-y)/eps;
    for(k=0; k<y.N; k++) JJ(k, i)=dy.elem(k);
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
//   MT::save(J, "z.J");
//   MT::save(JJ, "z.JJ");
  if(md>tolerance){
    MT_MSG("checkGradient (vector) -- FAILURE -- max diff=" <<md <<" (stored in files z.J_*)");
    MT::save(J, "z.J_analytical");
    MT::save(JJ, "z.J_emprical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
  }else{
    cout <<"checkGradient (vector) -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
}

uint optRprop(arr& x, ScalarFunction& f, double initialStepSize, double *fmin_return, double stoppingTolerance, uint maxEvals, uint verbose ){
  Rprop rp;
  rp.init(initialStepSize);
  return rp.loop(x, f, fmin_return, stoppingTolerance, maxEvals, verbose);
}

uint optGaussNewton(arr& x, VectorFunction& f, double *fmin_return, double stoppingTolerance, uint maxEvals, double maxStepSize, uint verbose){
  double a=1.;
  double fx, fy;
  arr Delta, y;
  arr R(x.N, x.N), r(x.N);
  uint evals=0;
  
  //compute initial costs
  arr phi, J;
  f.fv(phi, &J, x);  evals++;
  fx = sumOfSqr(phi);
  if(verbose>1) cout <<"*** optGaussNewton: starting point x=" <<x <<" l(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(verbose>0) fil.open("z.gaussNewton");
  if(verbose>0) fil <<0 <<' ' <<fx <<' ' <<a <<endl;
  
  for(;;){
    //compute Delta
    arr tmp;
    innerProduct(R, ~J, J);  R.reshape(x.N, x.N);
    innerProduct(r, ~J, phi);
    
    lapack_Ainv_b_sym(Delta, R, -r);
    if(maxStepSize>0. && norm(Delta)>maxStepSize)
      Delta *= maxStepSize/norm(Delta);
    
    for(;;){
      y = x + a*Delta;
      f.fv(phi, &J, y);  evals++;
      fy = sumOfSqr(phi);
      if(verbose>1) cout <<"optGaussNewton " <<evals <<" \tprobing y=" <<y <<" \tl(y)=" <<fy <<" \t|Delta|=" <<norm(Delta) <<" \ta=" <<a;
      CHECK(fy==fy, "cost seems to be NAN: ly=" <<fy);
      if(fy <= fx) break;
      if(evals>maxEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
      //decrease stepsize
      a = .1*a;
      if(verbose>1) cout <<" - reject" <<endl;
    }
    if(verbose>1) cout <<" - ACCEPT" <<endl;
    
    //adopt new point and adapt stepsize
    x = y;
    fx = fy;
    a = pow(a, 0.5);
    if(verbose>0) fil <<evals <<' ' <<fx <<' ' <<a <<endl;
    
    //stopping criterion
    if(norm(Delta)<stoppingTolerance || evals>maxEvals) break;
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.gaussNewton' us 1:2 w l",NULL,true);
  return evals;
}

uint optGradDescent(arr& x, ScalarFunction& f, double initialStepSize, double *fmin_return, double stoppingTolerance, uint maxEvals, double maxStepSize, uint verbose){
  uint evals=0;
  arr y, grad_x, grad_y;
  double fx, fy;
  double a=initialStepSize;

  fx = f.fs(&grad_x, x);  evals++;
  if(verbose>1) cout <<"*** optGradDescent: starting point x=" <<x <<" f(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(verbose>0) fil.open("z.grad");
  if(verbose>0) fil <<0 <<' ' <<fx <<' ' <<a <<endl;
  
  grad_x /= norm(grad_x);

  for(;;){
    y = x - a*grad_x;
    fy = f.fs(&grad_y, y);  evals++;
    CHECK(fy==fy, "cost seems to be NAN: fy=" <<fy);
    if(verbose>1) cout <<evals <<" \tprobing y=" <<y <<" \tf(y)=" <<fy <<" \t|grad|=" <<norm(grad_x) <<" \ta=" <<a;

    if(fy <= fx){
      if(verbose>1) cout <<" - ACCEPT" <<endl;
      double step=norm(x-y);
      x = y;
      fx = fy;
      grad_x = grad_y/norm(grad_y);
      a *= 1.2;
      if(maxStepSize>0. && a>maxStepSize) a = maxStepSize;
      if(verbose>0) fil <<evals <<' ' <<fx <<' ' <<a <<endl;
      if(step<stoppingTolerance) break;
    }else{
      if(verbose>1) cout <<" - reject" <<endl;
      a *= .5;
    }
    if(evals>maxEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.grad' us 1:2 w l",NULL,true);
  return evals;
}


//===========================================================================
//
// Rprop
//

int _sgn(double x){ if(x > 0) return 1; if(x < 0) return -1; return 0; }
double _mymin(double x, double y){ return x < y ? x : y; }
double _mymax(double x, double y){ return x > y ? x : y; }


Rprop::Rprop(){
  incr   = 1.2;
  decr   = .33;
  dMax = 50;
  dMin = 1e-6;
  rMax = 0;
  delta0 = 1.;
}

void Rprop::init(double _delta0){
  stepSize.resize(0);
  lastGrad.resize(0);
  delta0 = _delta0;
}

bool Rprop::done(){
  double maxStep = stepSize(stepSize.maxIndex());
  return maxStep < incr*dMin;
}

void Rprop::step(double& w, const double& grad){
  static arr W, GRAD;
  W.referTo(&w, 1); GRAD.referTo(&grad, 1);
  step(W, GRAD);
}

void Rprop::step(arr& w, const arr& grad, uint *singleI){
  if(!stepSize.N){ //initialize
    stepSize.resize(w.N);
    lastGrad.resize(w.N);
    lastGrad.setZero();
    stepSize = delta0;
  }
  CHECK(grad.N==stepSize.N, "Rprop: gradient dimensionality changed!");
  CHECK(w.N==stepSize.N   , "Rprop: parameter dimensionality changed!");
  
  uint i=0, I=w.N;
  if(singleI){ i=*(singleI); I=i+1; }
  for(; i<I; i++){
    if(grad.elem(i) * lastGrad(i) > 0){  //same direction as last time
      if(rMax) dMax=fabs(rMax*w.elem(i));
      stepSize(i) = _mymin(dMax, incr * stepSize(i)); //increase step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    } else if(grad.elem(i) * lastGrad(i) < 0){  //change of direction
      stepSize(i) = _mymax(dMin, decr * stepSize(i)); //decrease step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = 0;                               //memorize to continue below next time
    }else{                               //after change of direcion
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    }
  }
}

void Rprop::step(arr& x, ScalarFunction& f){
  arr grad;
  f.fs(&grad, x);
  step(x, grad);
}

//----- the rprop wrapped with stopping criteria
uint Rprop::loop(arr& _x,
                 ScalarFunction& f,
                 double *fmin_return,
                 double stoppingTolerance,
                 uint maxEvals,
                 uint verbose){
  arr x, J(_x.N), xmin;
  double y, ymin=0;
  uint lost_steps=0, small_steps=0;
  x=_x;
  
  ofstream fil;
  if(verbose>0) fil.open("z.rprop");
  
  uint i;
  for(i=0; i<maxEvals; i++){
    //checkGradient(p, x, stoppingTolerance);
    //compute value and gradient at x
    y = f.fs(&J, x);
    //update best-so-far
    if(!i){ ymin=y; xmin=x; }
    if(y<ymin){
      ymin=y; xmin=x;
      lost_steps=0;
    }else{
      lost_steps++;
      if(lost_steps>10){
        stepSize*=(double).1;
        lastGrad=(double)0.;
        x=xmin;
        lost_steps=0;
      }
    }
    //update x
    step(x, J);
    //check stopping criterion based on step-length in x
    double diff=maxDiff(x, xmin);
    if(verbose>0) fil <<i <<' ' <<y <<' ' <<diff <<endl;
    if(verbose>1) cout <<"optRprop " <<i <<' ' <<y <<' ' <<diff <<endl;
    if(diff<stoppingTolerance){ small_steps++; }else{ small_steps=0; }
    if(small_steps>10)  break;
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.rprop' us 1:2 w l", NULL, true);
  if(fmin_return) *fmin_return=ymin;
  _x=xmin;
  return i;
}


#ifdef MT_GSL
#include <gsl/gsl_cdf.h>
bool DecideSign::step(double x){
  N++;
  sumX+=x;
  sumXX+=x*x;
  if(N<=10) return false;
  if(!sumX) return true;
  double m=sumX/N;
  double s=sqrt((sumXX-sumX*m)/(N-1));
  double t=sqrt(N)*fabs(m)/s;
  double T=gsl_cdf_tdist_Pinv(1.-1e-6, N-1); //decide with error-prob 1e-4 if sign is significant
  if(t>T) return true;
  return false;
}
#else
bool DecideSign::step(double x){ NIY; }
#endif
