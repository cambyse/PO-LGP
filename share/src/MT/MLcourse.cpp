#include "MLcourse.h"

arr beta_true;

double NormalSdv(const double& a, const double& b, double sdv){
  double d=(a-b)/sdv;
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*::exp(-.5*d*d);
}

void linearRegression(arr& beta, const arr& X, const arr& y, const arr* weighted){
  CHECK(y.nd==1 && X.nd==2 && y.N==X.d0, "wrong dimensions");
  arr Xt, XtXinv;
  transpose(Xt, X);
  if(weighted) Xt = Xt * diag(*weighted);
  inverse_SymPosDef(XtXinv, Xt*X);
  beta = XtXinv*(Xt*y);
}

void ridgeRegression(arr& beta, const arr& X, const arr& y, double lambda, const arr* weighted, arr* zScores){
  CHECK(y.nd==1 && X.nd==2 && y.N==X.d0, "wrong dimensions");
  arr Xt, XtXinv, I;
  transpose(Xt, X);
  I.setDiag(lambda, X.d1);
  I(0, 0)=1e-10; //don't regularize beta_0 !!
  if(weighted) Xt = Xt * diag(*weighted);
  inverse_SymPosDef(XtXinv, Xt*X + I);
  beta = XtXinv*(Xt*y);
  if(zScores){
    (*zScores).resize(beta.N);
    double sigma = sumOfSqr(X*beta-y)/(y.N - X.d1 - 1.);
    for(uint i=0; i<beta.N; i++){
      (*zScores)(i) = fabs(beta(i)) / (sigma * sqrt(XtXinv(i, i)));
    }
  }
}

void logisticRegression2Class(arr& beta, const arr& X, const arr& y, double lambda){
  CHECK(y.nd==1, "");
  uint n=y.N, d=X.d1;
  arr Xt;
  transpose(Xt, X);
  
  arr XtWXinv, I;
  I.setDiag(lambda, X.d1);
  //I(0, 0)=1e-10; on classification is makes sense to include the bias in regularization, I think... (rescaling ov beta only changes the slope of the sigmoid, not the decision boundary)
  
  arr f(n), p(n), w(n), beta_update;
  double logLike, lastLogLike, alpha=1.;
  beta.resize(d);
  beta.setZero();
  for(uint k=0; k<100; k++){
    f = X*beta;
    logLike=0.;
    for(uint i=0; i<n; i++){
      if(f(i)<-100.) f(i)=-100.;  if(f(i)>100.) f(i)=100.;  //constrain the discriminative values to avoid NANs...
      p(i) = 1./(1.+exp(-f(i)));
      w(i) = p(i)*(1.-p(i));
      logLike += y(i)*log(p(i))+(1.-y(i))*log(1.-p(i));
      //more numeric robust way to compute likelihood by Tim
    }
    
    //optionally reject the update
    if(k && logLike<lastLogLike){
      //cout <<"REJECT" <<endl;
      beta -= alpha*beta_update;
      alpha *= .1;
      beta += alpha*beta_update;
      if(alpha*beta_update.absMax()<1e-5) break;
      continue;
    }else{
      alpha = pow(alpha, .8);
    }
    lastLogLike=logLike;
    
    inverse_SymPosDef(XtWXinv, Xt*diagProduct(w, X) + 2.*I);        //inverse Hessian
    beta_update = XtWXinv * (Xt*(y-p) - 2.*I*beta);   //beta update equation
    beta += alpha*beta_update;
    
    cout <<"logReg iter= " <<k <<" logLike= " <<logLike/n <<" beta_update= " <<beta_update.absMax() <<" alpha= " <<alpha <<endl;
    
    if(alpha*beta_update.absMax()<1e-5) break;
  }
}

void logisticRegressionMultiClass(arr& beta, const arr& X, const arr& y, double lambda){
  CHECK(y.nd==2 && y.d0==X.d0, "");
  uint n=y.d0, d=X.d1, M=y.d1;
  arr Xt;
  transpose(Xt, X);
  
  arr XtWX, XtWXinv, I;
  I.setDiag(lambda, X.d1);
  I(0, 0)=1e-10;
  
  arr f(n, M), p(n, M), w(n), beta_update;
  double logLike, lastLogLike, alpha=1.;
  beta.resize(d, M);
  beta.setZero();
  for(uint k=0; k<100; k++){
    f = X*beta;
    for(uint i=0; i<f.N; i++) MT::constrain(f.elem(i), -100., 100);  //constrain the discriminative values to avoid NANs...
    p = exp(f);
    logLike=0.;
    for(uint i=0; i<n; i++){
      p[i]() /= sum(p[i]); //normalize the exp(f(x)) along each row
      for(uint c=0; c<M; c++) logLike += y(i, c)*log(p(i, c));
    }
    
    //optionally reject the update
    if(k && logLike<lastLogLike){
      //cout <<"REJECT" <<endl;
      beta -= alpha*beta_update;
      alpha *= .1;
      beta += alpha*beta_update;
      if(alpha*beta_update.absMax()<1e-5) break;
      continue;
    }else{
      alpha = pow(alpha, .8);
    }
    lastLogLike=logLike;
    
    //construct the Hessian matrix as block matrix of size d*M-times-d*M (the beta is of size d*M)
    XtWX.resize(beta.N, beta.N);
    XtWX.setZero();
    for(uint c1=0; c1<M; c1++) for(uint c2=0; c2<M; c2++){
        for(uint i=0; i<n; i++) w(i) = p(i, c1)*((c1==c2?1.:0.)-p(i, c2));
        XtWX.setMatrixBlock(Xt*diagProduct(w, X) + (c1==c2?2.:0.)*I, c1*d, c2*d);
      }
    inverse_SymPosDef(XtWXinv, XtWX);
    //compute the beta update
    beta_update = (Xt*(y-p) - 2.*I*beta); //the gradient as d-times-M matrix
    beta_update = ~beta_update;           //... as M-times-d matrix
    beta_update.reshape(d*M);             //... as one big vector
    beta_update = XtWXinv * beta_update;  //multiply the Hessian
    beta_update.reshape(M, d);             //... as M-times-d matrix
    beta_update = ~beta_update;           //... and back as d-times-M matrix
    
    beta += alpha*beta_update;
    
    cout <<"logReg iter= " <<k <<" logLike= " <<logLike/n <<" beta_update= " <<beta_update.absMax() <<" alpha= " <<alpha <<endl;
    if(alpha*beta_update.absMax()<1e-5) break;
  }
}

void CrossValidation::crossValidate(const arr& X, const arr& y, double lambda, uint k_fold, bool permute, double *scoreMean, double *scoreSDV, double *scoreTrain){
  arr Xtrain, Xtest, ytrain, ytest;
  uint n=X.d0, blocksize;
  CHECK(y.N==n, "");
  CHECK(!(n%k_fold), "data size (" <<n <<") must be multiple of k_fold (" <<k_fold <<")");
  blocksize = n/k_fold;
  double cost, costM=0., costD=0.;
  arr X_perm, y_perm;
  if(permute){
    uintA perm;
    perm.setRandomPerm(X.d0);
    X_perm=X;  X_perm.permuteRows(perm);
    y_perm=y;  y_perm.permute(perm);
  }
  for(uint k=0; k<k_fold; k++){
    if(!permute){
      Xtrain = X;  ytrain = y;
    }else{
      Xtrain = X_perm;  ytrain = y_perm;
    }
    Xtrain.delRows(k*blocksize, blocksize);
    ytrain.remove(k*blocksize, blocksize);
    Xtest.referToSubRange(X, k*blocksize, (k+1)*blocksize-1);
    ytest.referToSubRange(y, k*blocksize, (k+1)*blocksize-1);
    
    train(Xtrain, ytrain, lambda);
    cost = test(Xtest, ytest);
    costM += cost;
    costD += cost*cost;
  }
  costM /= k_fold;
  costD /= k_fold;
  costD -= costM*costM;
  costD = sqrt(costD)/sqrt(k_fold); //sdv of the mean estimator
  
  //on full training data:
  train(X, y, lambda);
  double costT = test(X, y);
  
  if(scoreMean)  *scoreMean =costM; else scoreMeans =ARR(costM);
  if(scoreSDV)   *scoreSDV  =costD; else scoreSDVs  =ARR(costD);
  if(scoreTrain) *scoreTrain=costT; else scoreTrains=ARR(costT);
  cout <<"CV: lambda=" <<lambda <<" \tmean=" <<costM <<" \tsdv=" <<costD <<" \ttrain=" <<costT <<endl;
}

void CrossValidation::crossValidate(const arr& X, const arr& y, const arr& _lambdas, uint k_fold, bool permute){
  lambdas=_lambdas;
  scoreMeans.resizeAs(lambdas);
  scoreSDVs.resizeAs(lambdas);
  scoreTrains.resizeAs(lambdas);
  for(uint i=0; i<lambdas.N; i++){
    crossValidate(X, y, lambdas(i), k_fold, permute, &scoreMeans(i), &scoreSDVs(i), &scoreTrains(i));
  }
}

void CrossValidation::plot(){
  write(LIST(lambdas, scoreMeans, scoreSDVs, scoreTrains), "cv");
  gnuplot("set log x; set xlabel 'lambda'; set ylabel 'mean squared error'; plot 'cv' us 1:2:3 w errorlines title 'cv error','cv' us 1:4 w l title 'training error'", "z.pdf", true);
  
}

void linearFeatures(arr& Z, const arr& X){
  Z.setBlockMatrix(ones(TUP(X.d0, 1)), X);
}

void quadraticFeatures(arr& Z, const arr& X){
  uint n=X.d0, d=X.d1;
  Z.resize(n, 1 + d + d*(d+1)/2);
  uint i, j, k, l;
  for(i=0; i<n; i++){
    arr x=X[i];
    arr z=Z[i];
    l=0;
    z(l++)=1.;
    for(j=0; j<d; j++) z(l++) = x(j);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) z(l++) = x(j)*x(k);
  }
}

void cubicFeatures(arr& Z, const arr& X){
  uint n=X.d0, d=X.d1;
  Z.resize(n, 1 + d + d*(d+1)/2 + d*(d+1)*(d+2)/6);
  uint i, j, k, l, m;
  for(i=0; i<n; i++){
    arr x=X[i];
    arr z=Z[i];
    l=0;
    z(l++)=1.;
    for(j=0; j<d; j++) z(l++) = x(j);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) z(l++) = x(j)*x(k);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) for(m=0; m<=k; m++) z(l++) = x(j)*x(k)*x(m);
  }
}

void piecewiseConstantFeatures(arr& Z, const arr& X){
  uint n=X.d0, d=X.d1;
  if(d!=1) HALT("only for 1D data");
  Z.resize(n, 6);
  Z.setZero();
  for(uint i=0; i<n; i++){
    double x=X(i, 0);
    arr z=Z[i];
    if(x<-2.5) x=-2.5; if(x>2.5) x=2.5;
    z(floor(x+3.))=1.;
  }
}

void piecewiseLinearFeatures(arr& Z, const arr& X){
  uint n=X.d0, d=X.d1;
  if(d!=1) HALT("only for 1D data");
  Z.resize(n, 7);
  Z.setZero();
  for(uint i=0; i<n; i++){
    double x=X(i, 0);
    arr z=Z[i];
    z(0) = 1.; //constant
    z(1) = x; //linear
    for(int j=-2; j<=2; j++) z(j+4) = x<j?0:x-j;
  }
}


void rbfFeatures(arr& Z, const arr& X, const arr& Xtrain, double sigma){
  int rbfConst = MT::getParameter<int>("rbfConst", 0);
  Z.resize(X.d0, Xtrain.d0+rbfConst);
  for(uint i=0; i<Z.d0; i++){
    if(rbfConst) Z(i, 0) = 1.; //constant feature also for rbfs
    for(uint j=0; j<Xtrain.d0; j++){
      double d=euclideanDistance(X[i], Xtrain[j])/sigma;
      Z(i, j+rbfConst) = ::exp(-.5*d*d);
    }
  }
}

void makeFeatures(arr& Z, const arr& X, const arr& Xtrain, FeatureType featureType){
  if(featureType==readFromCfgFileFT) featureType = (FeatureType)MT::getParameter<uint>("modelFeatureType", 1);
  switch(featureType){
    case linearFT:    linearFeatures(Z, X);  break;
    case quadraticFT: quadraticFeatures(Z, X);  break;
    case cubicFT:     cubicFeatures(Z, X);  break;
    case rbfFT:       rbfFeatures(Z, X, Xtrain, MT::getParameter<double>("rbfSigma", .2));  break;
    case piecewiseConstantFT:  piecewiseConstantFeatures(Z, X);  break;
    case piecewiseLinearFT:    piecewiseLinearFeatures(Z, X);  break;
    default: HALT("");
  }
}



void artificialData(arr& X, arr& y, ArtificialDataType dataType){
  uint n = MT::getParameter<uint>("n", 100);
  uint d = MT::getParameter<uint>("d", 1);
  double sigma = MT::getParameter<double>("sigma", 1.); // observation noise
  
  if(dataType==readFromCfgFileDT) dataType = (ArtificialDataType)MT::getParameter<uint>("dataType", 1);
  switch(dataType){
    case linearData: {
      X = randn(n, d);
      arr Z;
      makeFeatures(Z, X, X, (FeatureType)MT::getParameter<uint>("dataFeatureType", 1));
      arr beta;
      beta = randn(Z.d1, 1).reshape(Z.d1);
      y = Z*beta;
      y = y + sigma*randn(size(y));
      beta_true = beta;
      break;
    }
    case sinusData: {
      X.setGrid(1, -3, 3, n-1);
      y.resize(X.d0);
      for(uint i=0; i<X.d0; i++) y(i) = sin(X(i, 0));
      y += sigma*randn(size(y));
      break;
    }
    case linearOutlier: {
      double rate = MT::getParameter<double>("outlierRate", .1);
      X = randn(n, d);
      arr Z;
      makeFeatures(Z, X, X, (FeatureType)MT::getParameter<uint>("dataFeatureType", 1));
      arr beta;
      beta = randn(Z.d1, 1).reshape(Z.d1);
      y = Z*beta;
      for(uint i=0; i<y.N; i++)  if(rnd.uni()<rate){
          y(i) += MT::getParameter<double>("outlierSigma", 10.)*rnd.gauss();
        }else{
          y(i) += sigma*rnd.gauss();
        }
      beta_true = beta;
      break;
    }
    default: HALT("");
  }
}

void artificialData_Hasties2Class(arr& X, arr& y){
  uint n = MT::getParameter<uint>("n", 100);
  
  arr means0(10, 2), means1(10, 2), x(2);
  
  rndGauss(means0);  means0 += ones(10, 1)*~ARR(1, 0);
  rndGauss(means1);  means1 += ones(10, 1)*~ARR(0, 1);
  
  X.clear();
  y.clear();
  for(uint i=0; i<n; i++){
    rndGauss(x, .2);  x += means0[rnd(10)];
    X.append(~x);
    y.append(0);
    
    rndGauss(x, .2);  x += means1[rnd(10)];
    X.append(~x);
    y.append(1);
  }
}

void artificialData_HastiesMultiClass(arr& X, arr& y){
  uint n = MT::getParameter<uint>("n", 100);
  uint M = MT::getParameter<uint>("M", 3);
  
  arr means(M, 10, 2), x(2);
  
  rndGauss(means);
  for(uint c=0; c<M; c++)  means[c]() += ones(10, 1)*~ARR(c, c);
  
  X.resize(M*n, 2);
  y.resize(M*n, M);
  y.setZero();
  for(uint i=0; i<n; i++){
    for(uint c=0; c<M; c++){
      arr x=X[i*M+c];  rndGauss(x, .2);  x += means[c][rnd(10)];
      y(i*M+c, c)=1.;
    }
  }
}

void artificialData_GaussianMixture(arr& X, arr& y){
  uint n = MT::getParameter<uint>("n", 100);
  uint M = MT::getParameter<uint>("M", 3);
  double sig = MT::getParameter<double>("sigma", .2);
  
  arr means(M, 2), V(M, 2, 2), x(2);
  
  rndGauss(means);
  rndGauss(V);
  //means.setZero();
  //for(uint c=0;c<M;c++)  means[c]() += ARR(c, c);
  
  X.resize(M*n, 2);
  y.resize(M*n, M);
  y.setZero();
  for(uint i=0; i<n; i++){
    for(uint c=0; c<M; c++){
      arr x=X[i*M+c];  rndGauss(x, sig);  x = V[c]*x;  x += means[c];
      y(i*M+c, c)=1.;
    }
  }
}


void load_data(arr& X, const char* filename, bool whiten){
  ifstream is;
  MT::open(is, filename);
  MT::Array<MT::String> strs;
  if(!MT::contains("0123456789.-+", MT::peerNextChar(is))){
    //read line of strings
    MT::String str;
    for(;;){
      str.read(is, " \"\t\r", " \"\t\r\n", false);
      if(!str.N()) break;
      strs.append(str);
    }
    cout <<"header: " <<strs <<endl;
  }
  X.clear();
  X.read(is);
  cout <<"data stats:"
       <<"\n  data entries    n=" <<X.d0
       <<"\n  entry dimension d=" <<X.d1
       <<"\n  stats: [# 'name' mean sdv]" <<endl;
  arr mean = sum(X, 0);  mean /= (double)X.d0;
  arr var = ~X*X;       var /= (double)X.d0;
  var -= mean^mean;
  for(uint j=0; j<X.d1; j++){
    cout <<j <<' ';
    if(strs.N) cout <<strs(j) <<' ';
    cout <<mean(j) <<' ' <<sqrt(var(j, j)) <<endl;
  }
  
  //-- whiten the data
  if(whiten){
    for(uint i=0; i<X.d0; i++) for(uint j=0; j<X.d1; j++){
        X(i, j) /= sqrt(var(j, j));
      }
  }
}
