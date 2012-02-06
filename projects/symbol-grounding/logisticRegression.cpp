#include "logisticRegression.h"
#include "sampler.h"

#include <MT/array.h>
#include <JK/util.h>

class LogisticRegressionEvaluator : public Evaluator<MT::Array<arr> > {
  private:
    sLogisticRegression* p;
  public:
    LogisticRegressionEvaluator(sLogisticRegression* p) : p(p) {};
    double evaluate(MT::Array<arr>& sample);
};

class sLogisticRegression {
  public:
    sLogisticRegression() : evaluator(LogisticRegressionEvaluator(this)) {};
    arr beta;  

    arr data;
    arr classes;

    double ridge, sigma;
    int rbfConst;

    void makeFeatures(arr& Z, const arr& X, const arr& Xtrain );
    void logisticRegressionMultiClass(arr& beta, const arr& X, const arr& y, double lambda);

    LogisticRegressionEvaluator evaluator;
    Sampler<MT::Array<arr> >* sampler;
};

void flatten(arr& out, const MT::Array<arr>& in) {
  uint length = 0;
  for (uint i = 0; i < in.d0; ++i) {
    for (uint j = 0; j < in.d1; ++j) {
      out.append(in(i, j));  
      if (!i) {
         length += in(i,j).N;
      }
    }
  }
  out.reshape(in.d0, length);
}

void createClassMatrix(arr& matrix, const intA& classes, const uint nClasses) {
  for (uint i = 0; i < classes.d0; ++i) {
    arr row = zeros(1, nClasses);
    row(0, classes(i)) = 1;
    matrix.append(row);
  }
  matrix.reshape(classes.d0, nClasses);
  
}

double LogisticRegressionEvaluator::evaluate(MT::Array<arr>& sample) {
  arr Phi, x;
  flatten(x, sample);
  p->makeFeatures(Phi, x, p->data);
  arr classes = Phi*p->beta;
  //return 1;
  return -fabs(classes(0, 0));
}

LogisticRegression::LogisticRegression(Sampler<MT::Array<arr> >* sampler) : s(new sLogisticRegression) {
  s->sampler = sampler;
  s->ridge = MT::getParameter<double>("ridge",1e-10);
  s->rbfConst = MT::getParameter<int>("rbfConst", 0);
  s->sigma = MT::getParameter<double>("sigma", 1);
}

void sLogisticRegression::makeFeatures(arr& Z, const arr& X, const arr& Xtrain){
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

//void sLogisticRegression::makeFeatures(arr& Z, const arr& X, const arr& Xtrain ) {
  //Z.resize(X.d0, Xtrain.d0+rbfConst);
  //for(uint i=0; i<Z.d0; i++){
    //if(rbfConst) Z(i, 0) = 1.; //constant feature also for rbfs
    //for(uint j=0; j<Xtrain.d0; j++){
      //double d=euclideanDistance(X[i], Xtrain[j])/sigma;
      //Z(i, j+rbfConst) = std::exp(-.5*d*d);
    //}
  //}
//}

void sLogisticRegression::logisticRegressionMultiClass(arr& beta, const arr& X, const arr& y, double lambda){
  CHECK(y.nd==2 && y.d0==X.d0, "");
  // M is the number of classes. y contains a row for each class holding a 0
  // if X(i) is not in this class and a 1 if it is in that class.
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
    // rprop
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
    XtWX = zeros(beta.N);
    for(uint c1=0; c1<M; c1++) for(uint c2=0; c2<M; c2++){
        for(uint i=0; i<n; i++) w(i) = p(i, c1)*((c1==c2?1.:0.)-p(i, c2));
        XtWX.setMatrixBlock(Xt*diagProduct(w, X) + (c1==c2?2.:0.)*I, c1*d, c2*d);
      }
    inverse_SymPosDef(XtWXinv, XtWX);
    //inverse(XtWXinv, XtWX);
    //compute the beta update
    beta_update = (Xt*(y-p) - 2.*I*beta); //the gradient as d-times-M matrix
    beta_update = ~beta_update;           //... as M-times-d matrix
    beta_update.reshape(d*M);             //... as one big vector
    beta_update = XtWXinv * beta_update;  //multiply the Hessian
    beta_update.reshape(M, d);             //... as M-times-d matrix
    beta_update = ~beta_update;           //... and back as d-times-M matrix
    
    beta += alpha*beta_update;
    
    if(alpha*beta_update.absMax()<1e-5) break;
  }
}


void LogisticRegression::setTrainingsData(const MT::Array<arr>& data, const intA& classes) {
  arr X, x, Phi, matrix;
  
  flatten(x, data);
  createClassMatrix(matrix, classes, 2);

  s->classes = matrix;
  s->data = x;

  s->makeFeatures(Phi,x,x);
  s->logisticRegressionMultiClass(s->beta, Phi, matrix, s->ridge);
}
void LogisticRegression::addData(const MT::Array<arr>& data, const int class_) {
  arr x;
  flatten(x, data);
  s->data.append(x);
  arr matrix = zeros(1, s->classes.d1);
  matrix(0, class_) = 1;
  s->classes.append(matrix);

  arr Phi;
  s->makeFeatures(Phi, s->data, s->data);
  s->logisticRegressionMultiClass(s->beta, Phi, s->classes, s->ridge);
  
}
int LogisticRegression::nextSample(MT::Array<arr>& sample) const {
  rejectionSampling<MT::Array<arr> >(sample, s->sampler, &s->evaluator);
  return sample.N;
}
int LogisticRegression::classify(const MT::Array<arr>& data, const int set) const {
  arr Phi, x;
  flatten(x, data);
  s->makeFeatures(Phi, x, s->data);
  arr classes = Phi*s->beta;
  return classes.maxIndex();
}

