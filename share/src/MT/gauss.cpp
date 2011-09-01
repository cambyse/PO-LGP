#include "gauss.h"

#define takeC 1
#define takeU 2

bool useC=true;

int decideCU(const Gaussian &a, const Gaussian&b){
  CHECK(a.okU &&b.okU, "U should always be ok!");
  if(a.okU && b.okU){
    return takeU;
  }
  if(a.okC && b.okC){
    return takeC;
  }
  if(a.okC && !a.okU && !b.okC && b.okU){
    double Za=minDiag(a.C), Zb=minDiag(b.U);
    if(Za>Zb){ a.makeU(); return takeU; }else{ b.makeC(); return takeC; }
  }
  if(!a.okC && a.okU && b.okC && !b.okU){
    double Za=minDiag(a.U), Zb=minDiag(b.C);
    if(Za>Zb){ a.makeC(); return takeC; }else{ b.makeU(); return takeU; }
  }
  NIY;
  return 0;
}

void Gaussian::operator=(const Gaussian& x){
  if(x.okC){ c=x.c; C=x.C; }
  if(x.okU){ u=x.u; U=x.U; }
  okC=x.okC; okU=x.okU;
}

void Gaussian::makeU() const { //the const is actually not true -- but oh well..
  if(okU) return;
  CHECK(okC, "can't make canonical rep without normal rep");
  //CHECK(minDiag(C)>=1e-10, "inverting zero variance matrix: " <<C);
  for(uint i=0; i<C.d0; i++){ if(C(i, i)<1e-10) C(i, i)=1e-10; }
  Gaussian *g=(Gaussian*)this; //get a non-const pointer (hack :-()
  inverse(g->U, C);
  g->u=U*c;
  g->okU=true;
}

void Gaussian::makeC() const { //the const is actually not true -- but oh well..
  if(okC) return;
  CHECK(okU, "can't make canonical rep without normal rep");
  //CHECK(minDiag(U)>=1e-10, "inverting zero precision matrix:"  <<U);
  for(uint i=0; i<U.d0; i++){ if(U(i, i)<1e-10) U(i, i)=1e-10; }
  Gaussian *g=(Gaussian*)this; //get a non-const pointer (hack :-()
  inverse(g->C, U);
  g->c=C*u;
  g->okC=true;
}

void Gaussian::setConditional(uint n, double var1, double var2){
  if(useC){
    c.resize(2*n); c.setZero();
    arr d, dd; d.setDiag(var1, n); dd.setDiag(var1+var2, n);
    C.setBlockMatrix(d, d, d, dd);
    setCU(true, false);
  }else{
    u.resize(2*n); u.setZero();
    arr A, B, C;
    A.setDiag((var1+var2)/var1/var2, n);
    B.setDiag(1./var2, n);
    C.setDiag(-1./var2, n);
    U.setBlockMatrix(A, C, C, B);
    setCU(false, true);
  }
}

void Gaussian::setDiagonal(uint n, double var){
  if(useC){
    c.resize(n); c.setZero();
    C.setDiag(var, n);
    setCU(true, false);
  }else{
    u.resize(n); u.setZero();
    U.setDiag(1./var, n);
    setCU(false, true);
  }
}

void Gaussian::setConditional(const arr& f, const arr& F, const arr& Q){
  if(useC){
    Gaussian a;
    a.setDiagonal(F.d1, 1e+10);
    a.makeC();
    arr FA=F*a.C, tFA=~FA;
    C.setBlockMatrix(a.C, tFA, FA, Q + F*tFA);
    c.setBlockVector(a.c, F*a.c + f);
    setCU(true, false);
  }else{
    arr iQ, iQF, tFiQF, z(F.d1);
    inverse(iQ, Q);
    iQF=iQ*F;
    tFiQF=~F * iQF;
    iQF *= -1.;
    U.setBlockMatrix(tFiQF, ~iQF, iQF, iQ);
    z.setZero();
    u.setBlockVector(z, f);
    u = U * u;
    setCU(false, true);
  }
}

void Gaussian::write(std::ostream& os) const {
  if(!okU) MT_MSG("warning: Gaussian not in U form!");
  makeU();
  //os  <<"uU="  <<u  <<U;
  if(trace(U)<1e-10) os  <<"{<uniform Gaussian>}";
  else {
    makeC();
    os  <<"{ "  <<c  <<"\n  ";
    C.write(os, " ", "\n   ");
    os  <<" }";
  }
  return;
}

void Gaussian::read(std::istream& is){
  is >>c >>C;
  setCU(true, false);
}

void estimate(Gaussian& g, const arr& X){
  uint k=X.d0;
  double Z=(double)k;
  arr ones(k); ones=1.;
  g.c =ones*X;
  g.C =~X*X;
  g.C =(g.C  - (g.c^g.c)/Z)/Z;
  g.c /= Z;
  g.setCU(true, false);
}

void estimateWeighted(Gaussian& g, const arr& X, const arr& W){
  CHECK(X.d0==W.N, "");
  uint k=X.d0, n=X.d1, i;
  g.c.resize(n);   g.c.setZero();
  g.C.resize(n, n); g.C.setZero();
  double Z=0.;
  arr X_i;
  for(i=0; i<k; i++){
    Z   += W(i);
    X_i.referToSubDim(X, i);
    g.c += W(i) * X_i;
    g.C += W(i) * (X_i ^ X_i);
  }
  g.C =(g.C  - (g.c^g.c)/Z)/Z;
  g.c/=Z;
  g.setCU(true, false);
}

void collapseMoG(Gaussian& g, const arr& P, const GaussianA& G){
  if(useC){
    G(0).makeC();
    uint i, k=P.N, n=G(0).c.N;
    g.c.resize(n);   g.c.setZero();
    g.C.resize(n, n); g.C.setZero();
    arr CC(n, n);      CC.setZero();
    double Z=0.;
    for(i=0; i<k; i++){
      G(i).makeC();
      Z   += P(i);
      g.c += P(i) *  G(i).c;
      g.C += P(i) * (G(i).C + (G(i).c^G(i).c));
    }
    g.C -= (g.c^g.c)/Z;
    g.C /= Z;
    g.c /= Z;
    g.setCU(true, false);
  }else{
    NIY;
  }
}

void collapseMoG(Gaussian& g, const arr& P, const GaussianL& G, bool zeroMean){
  if(true || useC){
    G(0)->makeC();
    uint i, k=P.N, n=G(0)->c.N;
    g.c.resize(n);   g.c.setZero();
    g.C.resize(n, n); g.C.setZero();
    arr CC(n, n);      CC.setZero();
    double Z=0.;
    for(i=0; i<k; i++){
      G(i)->makeC();
      Z += P(i);
      if(!zeroMean) g.c += P(i) * G(i)->c;
      g.C += P(i) * (G(i)->C + (G(i)->c^G(i)->c));
    }
    g.C -= (g.c^g.c)/Z;
    g.C /= Z;
    g.c /= Z;
    g.setCU(true, false);
  }else{
    NIY;
  }
}

void resampleAndEstimate(Gaussian& g, double(*f)(const arr& x), uint N){
  doubleA X, W(N);
  double Z;
  uint i;
  sample(X, N, g);
  Z=0.;
  for(i=0; i<X.d0; i++){
    W(i) = f(X[i]);
    Z += W(i);
  }
  CHECK(Z>1e-10, "sample weights sum to approx. zero");
  estimateWeighted(g, X, W);
};

void sample(arr& x, const Gaussian& g){
  g.makeC();
  x.resize(g.C.d0);
  rndGauss(x, 1., false);
  arr U, V;
  svd(U, V, g.C);
  x = U*x;
  x += g.c;
}

void sample(arr& X, uint N, const Gaussian& g){
  uint i;
  X.resize(N, g.c.N);
  for(i=0; i<N; i++) sample(X[i](), g);
}

void systematicWeightedSamples(arr& X, arr& W, const Gaussian& g){
  uint n=g.c.N, i;
  
  //parameters
  double alpha=1., kappa=2.;
  double lambda=alpha*alpha*(n+kappa)-n;
  
  //standard samples
  g.makeC();
  X.resize(2*n+1, n);
  W.resize(X.d0);
  arr U, V, d(n), X_i;
  svd(U, V, g.C);
  X_i.referToSubDim(X, 0);        X_i=g.c;    W(0)=(lambda/(n+lambda));
  for(i=0; i<n; i++){
    d.setZero();
    d(i)=sqrt(n+lambda);
    d=U*d;
    //d=U.sub(0, -1, i, i); //i-th column
    //d.resize(n);
    X_i.referToSubDim(X, 2*i+1);  X_i=g.c+d;  W(2*i+1)=1./(2.*(n+lambda));
    X_i.referToSubDim(X, 2*i+2);  X_i=g.c-d;  W(2*i+2)=1./(2.*(n+lambda));
  }
}

void product(Gaussian& x, const Gaussian& a, const Gaussian& b, double *logNorm){
  if(useC){
    a.makeC(); b.makeC();
    arr X, AX, BX;
    inverse(X, a.C + b.C);
    if(logNorm){
      //cout  <<"computing norm:\na="  <<a.c  <<"b="  <<b.c  <<"\nX="  <<inverse(X)  <<endl;
      *logNorm = logNNinv(a.c, b.c, X);
    }
    AX = a.C*X;
    BX = b.C*X;
    x.C = AX*b.C;          //=A*X*B = A * (A+B)^-1 * B;
    x.c = BX*a.c + AX*b.c; //=B*X*a + A*X*b;
    x.setCU(true, false);
  }else{
    a.makeU(); b.makeU();
    x.U = a.U + b.U;
    if(logNorm){
      a.makeC();
      b.makeC();
      //cout  <<"computing norm:\na="  <<a.c  <<"b="  <<b.c  <<"\nX="  <<inverse(X)  <<endl;
      *logNorm = logNNinv(a.c, b.c, inverse(a.C + b.C));
    }
    x.u = a.u + b.u;
    x.setCU(false, true);
  }
}

void division(Gaussian& x, const Gaussian& a, const Gaussian& b, double *logNorm){
  if(useC){
    a.makeC(); b.makeC();
    arr X, AX, BX;
    X = inverse(a.C - b.C);  //X=(A-B)^-1
    AX = a.C*X;
    BX = b.C*X;
    x.C = AX*(-b.C);         //C=A*X*(-B);
    x.c = AX*b.c - BX*a.c;   //c=A*X*b - B*X*a;
    if(logNorm){
      a.makeC();
      b.makeC();
      *logNorm = - logNN(x.c, b.c, x.C+b.C);
    }
    x.setCU(true, false);
  }else{
    a.makeU(); b.makeU();
    x.U = a.U - b.U;
    x.u = a.u - b.u;
    x.setCU(false, true);
    if(logNorm){
      a.makeC();
      b.makeC();
      x.makeC();
      *logNorm = - logNN(x.c, b.c, x.C+b.C);
    }
  }
}

void forward(Gaussian& y, const Gaussian& x, arr& f, arr& F, arr& Q){
  //MT_MSG("needs U implementation");
  x.makeC();
  y.c = F*x.c + f;
  y.C = F*x.C*~F + Q;
  y.setCU(true, false);
}

void backward(Gaussian& x, const Gaussian& y, arr& f, arr& F, arr& Q, double updateStep){
  //MT_MSG("needs U implementation");
  CHECK(y.c.N==f.N && y.c.N==F.d0 && y.c.N==Q.d0, "linBwd: output dimension does not match lin fwd fct");
  y.makeC();
  arr Finv=inverse(F);
  if(F.d0>=F.d1){ //non-redundant map
    x.c = Finv*(y.c-f);
    x.C = Finv*(y.C+Q)*~Finv;
  }else{
    CHECK(x.c.N==F.d1, "x.c must be initialized when pulling back a redundant mapping!");
    //arr tF = ~F;
    //arr iF = tF * inverse(F*tF);
    //CHECK(maxDiff(Finv, iF)<1e-10, "pseudo inverse not right");
    arr D; D.setId(F.d1);
    x.c += updateStep*Finv*(y.c - (f + F*x.c));
    x.C = Finv*(y.C+Q)*~Finv;
    
    //strech the redundant dimensions
    arr U, w, V, Vt, vk;
    svd(U, w, V, F);
    double scale=norm(w);
    transpose(Vt, V);
    //cout  <<U  <<w  <<V;
    for(uint k=F.d0; k<w.N; k++){ //eigen vectors with zero eigen value
      CHECK(w(k)<1e-10*scale, "eigen value should be zero!");
      vk.referToSubDim(Vt, k);
      x.C += (1e+0*scale)* (vk^vk);
    }
  }
  x.setCU(true, false);
}

/*void forward(Gaussian& y, const Gaussian& x, Trans f, arr& Q){
  x.makeC();
  y=x;
  unscentedTransform(y.c, y.C, f);
  if(y.C.d0==2*Q.d0 && y.C.d1==2*Q.d1){ //assume joint is propagated
    arr zero(Q.d0, Q.d1); zero.setZero();
    arr block; blockMatrix(block, Q, zero, zero, Q);
    y.C += block;
  }else{
    y.C += Q;
  }
  y.okC=true; y.okU=false;
}*/

/*void fctFwd(Gaussian& y, Gaussian& x, Trans f){
  y=x;
  unscentedTransform(y.c, y.C, f);
}*/

void unscentedTransform(Gaussian &b, const Gaussian &a, Trans f){
  uint i, n=a.c.N;
  
  //following Murphy's http://www.cs.ubc.ca/~murphyk/Papers/dbnchapter.pdf
  
  //parameters
  double alpha=1., beta=0., kappa=2.;
  double lambda=alpha*alpha*(n+kappa)-n;
  
  //standard samples
  b.makeC();
  MT::Array<arr> X(2*n+1);
  arr U, V, d(n);
  svd(U, V, a.C);
  X(2*n)=a.c;
  for(i=0; i<n; i++){
    d=U.sub(0, -1, i, i); //i-th column
    d.resize(n);
    d*=sqrt(n+lambda);
    X(2*i)=a.c+d;
    X(2*i+1)=a.c-d;
  }
  
  //transform
  for(i=0; i<X.d0; i++) f(X(i));
  
  //weights
  double Wm0, Wc0, Wmc;
  Wm0=(lambda/(n+lambda));
  Wc0=(lambda/(n+lambda)) + (1-alpha*alpha+beta);
  Wmc=1./(2.*(n+lambda));
  
  //new means and covariance
  b.c.resize(n); b.C.resize(n, n);
  b.c = Wm0 * X(2*n);
  for(i=0; i<2*n; i++) b.c += Wmc * X(i);
  b.C = Wc0 * (X(2*n)-b.c)^(X(2*n)-b.c);
  for(i=0; i<2*n; i++) b.C += Wmc * (X(i)-b.c)^(X(i)-b.c);
  
#if 0
  std::cout  <<"unscent transform : "
             <<a  <<b  <<A  <<B
             <<"error : "  <<sqrDistance(a, b)  <<' '  <<sqrDistance(A, B)  <<std::endl;
            
  plotCovariance(a, A);
  plotPoints(X);
  plotFlush();
  
  //re-estimate
  arr b, B;
  estimate(X, b, B);
  //B*=double(2*n+1)/(2*n);
  
  //re-estimate
  estimate(X, a, A);
  A*=double(2*n+1)/(2*n); //to account for the mean as a sample point
#endif
}

void getLinFwdFromJoint(arr& f, arr& F, arr& Q, uint n1, uint n2, Gaussian& x){
  CHECK(n1+n2==x.c.N, "");
  arr A, B, C, Ainv, a, b;
  x.makeC();
  a=x.c.sub(0, n1-1);        //upper vector
  b=x.c.sub(n1, -1);         //lower vector
  A=x.C.sub(0, n1-1, 0, n1-1); //upper left
  B=x.C.sub(n1, -1, n1, -1);   //lower right
  C=x.C.sub(0, n1-1, n1, -1);  //upper right
  Ainv=inverse(A);
  
  F = ~C*Ainv;
  f = b - F*a;
  Q = B - F*C;
}

void getMarginal(Gaussian& x, const Gaussian& joint, uint dx){
  CHECK(dx < joint.N(), "doesn't make sense?");
  if(useC){
    joint.makeC();
    x.c=joint.c.sub(0, dx-1);        //sub vection
    x.C=joint.C.sub(0, dx-1, 0, dx-1); //sub block matrix
    x.setCU(true, false);
  }else{
    joint.makeU();
    uint na=dx, nb=joint.U.d0-na;
    arr A(na, na), B(nb, nb), C(na, nb);
    joint.U.getMatrixBlock(A, 0, 0);
    joint.U.getMatrixBlock(B, na, na);
    joint.U.getMatrixBlock(C, 0, na);
    arr iB, CiB;
    inverse(iB, B);
    CiB=C*iB;
    x.U = A - CiB*~C;
    arr a(na), b(nb);
    joint.u.getVectorBlock(a, 0);
    joint.u.getVectorBlock(b, na);
    x.u = a - CiB*b;
    x.setCU(false, true);
  }
}

void getMarginal(Gaussian& x, const Gaussian& joint, uintA& list){
  uint n=list.N, i, j;
  if(joint.okC){
    x.c.resize(n);
    x.C.resize(n, n);
    for(i=0; i<n; i++){
      x.c(i)=joint.c(list(i));
      for(j=0; j<n; j++) x.C(i, j)=joint.C(list(i), list(j));
    }
    x.setCU(true, false);
  }else{
    //--- first just extract the components u=(a, b) and U=(A, C;~C, B)
    
    //set permutation pi complementary to list
    uintA pi;
    if(joint.N()>list.N){
      pi.setStraightPerm(joint.N());
      for(i=0; i<list.N; i++) pi.permute(i, list(i));
      pi = pi.sub(n, -1);
    }else{
      pi.resize(0);
    }
    uint m=pi.N;
    
    //extract components
    arr A(n, n), B(m, m), C(n, m), a(n), b(m);
    for(i=0; i<n; i++){
      a(i)=joint.u(list(i));
      for(j=0; j<n; j++) A(i, j)=joint.U(list(i), list(j));
    }
    for(i=0; i<m; i++){
      b(i)=joint.u(pi(i));
      for(j=0; j<m; j++) B(i, j)=joint.U(pi(i), pi(j));
    }
    for(i=0; i<n; i++) for(j=0; j<m; j++) C(i, j)=joint.U(list(i), pi(j));
    
    //--- now it's easy:
    arr CBinv=C*inverse(B);
    x.u = a - CBinv * b;
    x.U = A - CBinv * ~C;
    x.setCU(false, true);
  }
}

void setMarginal(Gaussian& joint, const Gaussian& marg, uintA& list){
  uint n=list.N, i, j;
  marg.makeC();
  joint.makeC();
  if(marg.okC){
    CHECK(marg.c.N==n && marg.C.d0==n, "");
    for(i=0; i<n; i++){
      joint.c(list(i))=marg.c(i);
      for(j=0; j<n; j++) joint.C(list(i), list(j))=marg.C(i, j);
    }
    joint.setCU(true, false);
  }else{
    marg.makeC();
    CHECK(marg.c.N==n && marg.C.d0==n, "");
    for(i=0; i<n; i++){
      joint.c(list(i))=marg.c(i);
      for(j=0; j<n; j++) joint.C(list(i), list(j))=marg.C(i, j);
    }
    joint.setCU(true, false);
  }
}

void getMarginalsFromJoint(Gaussian& x, Gaussian& y, Gaussian& xi){
  xi.makeC();
  uint n=xi.c.N/2;
  CHECK(xi.c.N=n+n, "");
  x.c=xi.c.sub(0, n-1);        //upper vector
  y.c=xi.c.sub(n, -1);         //lower vector
  x.C=xi.C.sub(0, n-1, 0, n-1); //upper left
  y.C=xi.C.sub(n, -1, n, -1);   //lower right
  x.setCU(true, false);
  y.setCU(true, false);
}

void joinMarginalAndConditional(Gaussian& xi, Gaussian& a, arr& f, arr& F, arr& Q){
  a.makeC();
  arr FA=F*a.C, tFA=~FA;
  xi.C.setBlockMatrix(a.C, tFA, FA, Q + F*tFA);
  xi.c.setBlockVector(a.c, F*a.c + f);
  xi.setCU(true, false);
}

void getConditional(const Gaussian& xi, uint dx, arr& f, arr& F, arr& Q){
  if(useC){
    arr A, B, C, a, b;
    xi.makeC();
    uint k=dx, l=xi.C.d0-k;
    A.resize(k, k);
    B.resize(l, l);
    C.resize(k, l);
    xi.C.getMatrixBlock(A, 0, 0);
    xi.C.getMatrixBlock(B, k, k);
    xi.C.getMatrixBlock(C, 0, k);
    
    a.resize(k);
    b.resize(l);
    xi.c.getVectorBlock(a, 0);
    xi.c.getVectorBlock(b, k);
    
    F = ~C * inverse(A);
    f = b - F*a;
    Q = B - F*C;
  }else{
    xi.makeU();
    uint k=dx, l=xi.U.d0-k;
    arr A(k, k), B(l, l), C(k, l), a(k), b(l);
    xi.U.getMatrixBlock(A, 0, 0);
    xi.U.getMatrixBlock(B, k, k);
    xi.U.getMatrixBlock(C, 0, k);
    xi.u.getVectorBlock(a, 0);
    xi.u.getVectorBlock(b, k);
    
    Q = inverse(B);
    f = Q*b;
    F = -Q*~C;
  }
}

//! given P(x, y), return P(y|x)*UU(x) where UU(x) is uniform
void makeConditional(Gaussian& xi, uint dx){
  if(useC){
    NIY;
    arr A, B, C, a, b;
    xi.makeC();
    uint k=dx, l=xi.C.d0-k;
    A.resize(k, k);
    B.resize(l, l);
    C.resize(k, l);
    xi.C.getMatrixBlock(A, 0, 0);
    xi.C.getMatrixBlock(B, k, k);
    xi.C.getMatrixBlock(C, 0, k);
    
    a.resize(k);
    b.resize(l);
    xi.c.getVectorBlock(a, 0);
    xi.c.getVectorBlock(b, k);
  }else{
    xi.makeU();
    uint k=dx, l=xi.C.d0-k;
    arr B(l, l), C(k, l), b(l);
    //xi.U.getMatrixBlock(A, 0, 0);
    xi.U.getMatrixBlock(B, k, k);
    xi.U.getMatrixBlock(C, 0, k);
    //xi.u.getVectorBlock(a, 0);
    xi.u.getVectorBlock(b, k);
    
    arr CiB;
    CiB=C*inverse(B);
    xi.U.setMatrixBlock(CiB*~C, 0, 0);
    xi.u.setVectorBlock(CiB*b , 0);
  }
}

//simple version with first product rule
void multiplyToJoint(Gaussian& xi, Gaussian& b){
  uint n=b.N();
  CHECK(xi.N()==2*n, "only for doubles sizes yet");
  if(useC){
    arr zero(n, n); zero.setZero();
    arr big; big.setDiag(1e8, n);
    Gaussian B;
    B.c.setUni(0., n); B.c.append(b.c);
    B.C.setBlockMatrix(big, zero, zero, b.C);
    product(xi, xi, B);
  }else{
    NIY;
  }
}

/* version with 2nd product rule
void multiplyToJoint(Gaussian& xi, Gaussian& b){
  uint i, j, n2=b.c.N, n1=xi.c.N-b.c.N;
  arr Ainv, Binv, Cinv, ac=xi.c;
  Ainv=inverse(xi.C);
  Binv=inverse(b.C);
  //construct sum of both
  Cinv=Ainv;
  for(i=0;i<n2;i++) for(j=0;j<n2;j++){ //loop over the lower right blockmatrix
    Cinv(n1+i, n1+j) += Binv(i, j);
  }
  xi.C = inverse(Cinv);
  xi.c.resize(n1);
  xi.c.setZero();
  xi.c.append(Binv*b.c); //now is a block vector xi.c = [0 Binv*b]
  xi.c = xi.C * (Ainv * ac + xi.c);
}
*/

bool sameGaussian(const Gaussian &a, const Gaussian &b, double eps){
#if 1
  if(useC){
    return (maxDiff(a.C, b.C, 0)<eps) && (maxDiff(a.c, b.c, 0)<eps);
  }else{
    return (maxDiff(a.U, b.U, 0)<eps) && (maxDiff(a.u, b.u, 0)<eps);
  }
#else
  double d=KLDsym(a, b);
  //DEBUGINF(cout  <<"KLD = "  <<d  <<endl;);
  return d<1.;
#endif
}

double KLDsym(const Gaussian &a, const Gaussian &b){
  double x=0.;
  a.makeU(); b.makeU();
  a.makeC(); b.makeC();
  x = trace(b.U*a.C) + trace(a.U*b.C) + sqrDistance(a.U+b.U, a.c, b.c) - (2.*a.c.N);
  x /= 4.;
  return x;
}

double KLD(const Gaussian &a, const Gaussian &b){
  double x=0.;
  a.makeU(); b.makeU();
  a.makeC(); b.makeC();
  x = log(lapack_determinantSymPosDef(b.C)/lapack_determinantSymPosDef(a.C))
      + trace(b.U*a.C) + sqrDistance(b.U, a.c, b.c) - double(a.c.N);
  x /= 2.;
  return x;
}

void write(GaussianA& x, const char* name){
  ofstream os;
  MT::open(os, name);
  MT::IOraw=true;
  for(uint i=0; i<x.N; i++){
    x(i).makeC();
    os  <<x(i).c  <<" \t ";
    x(i).C.write(os, " ", " \t ");
    os  <<endl;
  }
}

void blowupMarginal(Gaussian& joint, const Gaussian& x, uint dy){
  if(useC){
    NIY;
  }else{
    uint dx=x.N();
    arr dd(dy, dy), d(dx, dy); d.setZero(); dd.setZero();
    arr z(dy); z.setZero();
    joint.U.setBlockMatrix(x.U, d, ~d, dd);
    joint.u.setBlockVector(x.u, z);
    joint.setCU(false, true);
  }
}

double reduce(GaussianA& g, uint m, const GaussianA& f, const arr& P, bool linearInit){
  uint i, j, n=P.N;
  g.resize(m);
  uintA pi(n), pi_old;
  
  cout  <<"** Gaussian reduction "  <<flush;
  //randomly initialize pi
  uintA perm;
  arr D(n, m);
  if(!linearInit){
    perm.setRandomPerm(n);
    perm.resizeCopy(m);
    for(j=0; j<m; j++) g(j).c=f(perm(j)).c; //randomly pick m centers
    //cout  <<perm  <<endl;
    //REGROUP
    for(i=0; i<n; i++) for(j=0; j<m; j++) D(i, j)=sqrDistance(f(i).c, g(j).c);
    //cout  <<D  <<endl;
    for(i=0; i<n; i++) pi(i) = D[i].minIndex();
    //cout  <<pi  <<endl;
  }else{
    for(i=0; i<n; i++) pi(i) = (i*m)/(n+1);
    //cout  <<pi  <<endl;
  }
  
  double score;
  uint k;
  for(k=0; k<30; k++){
    pi_old=pi;
    //REFIT
    MT::Array<GaussianL> group(m);
    MT::Array<arr> groupP(m);
    MT::Array<uintA> groupI(m);
    for(j=0; j<m; j++){ group(j).clear(); groupP(j).clear(); groupI(j).clear(); }
    for(i=0; i<n; i++){ j=pi(i); group(j).append(&f(i)); groupP(j).append(P(i)); groupI(j).append(i); }
    //cout  <<groupI  <<endl;
    for(j=0; j<m; j++){
      if(!groupP(j).N) return -1.;
      collapseMoG(g(j), groupP(j), group(j));
    }
    
    //REGROUP
    for(i=0; i<n; i++) for(j=0; j<m; j++) D(i, j)=KLD(f(i), g(j));
    //cout  <<D  <<endl;
    for(i=0; i<n; i++) pi(i) = D[i].minIndex();
    //cout  <<pi  <<endl;
    
    cout  <<'.'  <<flush;
    
    score=0.;
    for(i=0; i<n; i++) score += P(i) * D(i, pi(i));
    //cout  <<score  <<"  "  <<flush;
    if(pi==pi_old) break;
  }
  cout  <<" -> iterations="  <<k  <<" score="  <<score  <<endl;
  /*plotGnuplot();
  plotClear();
  plotGaussians(f);
  plot();
  plotGaussians(g);
  plot();*/
  return score;
}

double reduceIterated(GaussianA& g, uint m, const GaussianA& f, const arr& P, uint K){
  double s, best=-1.;
  arr S;
  GaussianA gk;
  for(uint k=0; k<K; k++){
    s=reduce(gk, m, f, P, false);
    S.append(s);
    if(s!=-1.) if(best==-1. || s<best){ best=s; g=gk; }
  }
  std::sort(S.p, S.pstop);
  cout  <<"scores = "  <<S  <<endl;
  cout  <<"best   = "  <<best  <<endl;
  return best;
}



#if 0
//===========================================================================
//
// older but correct versions
//


void equal(arr& c, arr& C, arr& a, arr& A){
  c=a;
  C=A;
}

void unscentedTransform(arr& a, arr& A, Trans f);

//! multiplication: {a, A} * {b, B} = {c, C} * norm
void product(arr& c, arr& C, arr& a, arr& A, arr& b, arr& B, double *norm=0){
  CHECK(&C != &B, "only C and A can be the same matrix");
  arr X, Y;
  X = inverse(A + B);
  Y = A*X;
  C = Y*B;   //=A*X*B;
  c = B*X*a + Y*b; //=B*X*a + A*X*b;
  if(norm){
    HALT("compute also normalization contant depending on a and b");
  }
}

//! given x~{a, A} and y|x~{Fx+f, Q}, this returns y~{c, C}
void linFwd(arr& c, arr& C, arr& a, arr& A, arr& f, arr& F, arr& Q){
  c = F*a + f;
  C = F*A*~F + Q;
}

//! given x~{a, A} and y|x~{f(x), Q}, this returns y~{c, C}
void fctFwd(arr& c, arr& C, arr& a, arr& A, Trans f, arr& Q){
  c=a; C=A;
  unscentedTransform(c, C, f);
  if(C.d0==2*Q.d0 && C.d1==2*Q.d1){ //assume joint is propagated
    arr zero(Q.d0, Q.d1); zero.setZero();
    arr block;
    block.setBlockMatrix(Q, zero, zero, Q);
    C += block;
  }else{
    C += Q;
  }
}

//! given y~{b, B} and y|x~{Fx+f, Q}, this returns x~{c, C}
void linBwd(arr& c, arr& C, arr& b, arr& B, arr& f, arr& F, arr& Q){
  arr Finv=inverse(F);
  c = Finv*(b-f);
  C = Finv*(B+Q)*~Finv;
}

/*! given x~{a, A} and y|x~{Fx+f, Q} and given evidence for y,
    this returns a potential U(x) = {c, C} such that x|y~{c, C}*P(x) given a prior P(x) */
void LinGaussPotential(arr& c, arr& C, arr& y, arr& f, arr& F, arr& Q){
  arr Finv=inverse(F);
  C = Finv*Q*~Finv;
  c = Finv*(y-f);
}

/*! given x~{a, A} and y|x~{f(x), Q} and given evidence for y,
    this returns a potential U(x) = {c, C} such that x|y~{c, C}*P(x) given a prior P(x) */
void FctGaussPotential(arr& c, arr& C, arr& y, Trans b, arr& Q){
  c=y; C=Q;
  unscentedTransform(c, C, b);
}

//! given x~{a, A} and y|x~{Fx+f, Q}, this returns the joint (x, y)~{c, C}
void jointFwd(arr& c, arr& C, arr& a, arr& A, arr& f, arr& F, arr& Q){
  arr FA=F*A, tFA=~FA;
  C.setBlockMatrix(A, tFA, FA, Q + F*tFA);
  c.setBlockVector(a, F*a + f);
}

/*! given a joint (x, y)~{c, C} and a potential {b, B} for y|evidence,
    this returns the updated joint (x, y)|evidence~{d, D} */
void multiplyToJoint(arr& d, arr& D, arr& c, arr& C, arr& b, arr& B){
  uint i, j, n2=b.N, n1=c.N-b.N;
  arr Cinv, Binv, Dinv, tmp;
  Cinv=inverse(C);
  Binv=inverse(B);
  //construct sum of both
  Dinv=Cinv;
  for(i=0; i<n2; i++) for(j=0; j<n2; j++){ //loop over the lower right blockmatrix
      Dinv(n1+i, n1+j) += Binv(i, j);
    }
  D = inverse(Dinv);
  d.resize(n1);
  d.setZero();
  d.append(Binv*b); //now is a block vector d = [0 Binv*b]
  d = D * (Cinv * c + d);
}

void getLinFwdFromJoint(arr& f, arr& F, arr& Q, uint n1, uint n2, arr& j, arr& J){
  CHECK(n1+n2==j.N, "");
  arr A, B, C, Ainv, a, b;
  a=j.sub(0, n1-1);        //upper vector
  b=j.sub(n1, -1);         //lower vector
  A=J.sub(0, n1-1, 0, n1-1); //upper left
  B=J.sub(n1, -1, n1, -1);   //lower right
  C=J.sub(0, n1-1, n1, -1);  //upper right
  Ainv=inverse(A);
  
  F = ~C*Ainv;
  f = b - F*a;
  Q = B - F*C;
}

void estimate(arr& X, arr& mean, arr& cov){
  uint k=X.d0;
  arr ones(k); ones=1.;
  mean=ones*X;
  cov =~X*X;
  cov =(cov  - (mean^mean)/(double)k)/double(k);
  mean/=(double)k;
}

void sampleGauss(arr& x, arr& A){
  x.resize(A.d0);
  rndGauss(x);
  arr U, V;
  svd(U, V, A);
  x=U*x;
}
void sampleGauss(arr& x, arr& a, arr& A){
  CHECK(A.nd==2 && A.d0==A.d1 && a.nd==1 && a.N==A.d0, "");
  sampleGauss(x, A);
  x+=a;
}

void sampleGauss(arr& X, arr& a, arr& A, uint N){
  uint i;
  X.resize(N, a.N);
  for(i=0; i<N; i++) sampleGauss(X[i](), a, A);
}

#endif
