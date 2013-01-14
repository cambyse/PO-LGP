#include "gp_ams.h"

#include <JK/gpc.h>
#include <JK/util.h>
#include <MT/gaussianProcess.h>

#include <cmath>

gp_classification::gp_classification(const gp_classification& r) {
  gp = new GPC(*r.gp);  
}
gp_classification::gp_classification(GPC* gpc) {
  gp = gpc;  
}
gp_classification::~gp_classification() {
  delete gp;
}
const double gp_classification::log_likelihood() const {
  return gp->log_likelihood();  
}
const double gp_classification::prediction(const arr& x, const double y) const {
  double mean, sig;
  gp->class_probability(mean, sig, x);
  double g=gauss(mean,sig,y);
  if (g<10e-20) g=10e-20;
  return g;
}
void gp_classification::add_observation(const arr& x, const double y) {
  gp->addData(x, y);
}
void gp_classification::modes(double& mean, double& var, const arr& x) {
  gp->class_probability(mean, var, x);
}
ams_model* gp_classification::clone() const {
  return new gp_classification(*this);  
}

gp_regression::gp_regression(const gp_regression& r) {
  gp = new GaussianProcess(*r.gp);  
}
gp_regression::gp_regression(GaussianProcess* gpr) {
  gp = gpr;  
}
gp_regression::~gp_regression() {
  delete gp;  
}
const double gp_regression::log_likelihood() const {
  return gp->log_likelihood();  
}
const double gp_regression::prediction(const arr& x, const double y) const {
  double mean, sig;
  gp->evaluate(x, mean, sig);
  double g=gauss(mean,sig,y);
  if (g<10e-20) g=10e-20;
  return g;
}
void gp_regression::add_observation(const arr& x, const double y) {
  gp->appendObservation(x, y);
  gp->recompute();  
}
void gp_regression::modes(double& mean, double& var, const arr& x) {
  gp->evaluate(x, mean, var);
}
ams_model* gp_regression::clone() const {
  return new gp_regression(*this);  
}


double kullback_leibler_divergence(const arr& mu1, const arr& sigma1, const arr& mu2, const arr& sigma2) {
  return .5*( log(determinant(sigma2)/determinant(sigma1)) + trace(inverse(sigma2)*sigma1) + (~(mu1 - mu2)*inverse(sigma2)*(mu1-mu2))(0) - mu1.d0 );
}

double sum_of_dkl(const model_list_t& models, const arr& x_s) {
  arr means(models.N);
  arr var(models.N);
	uint i=0; ams_model* m;
  for_list(i, m, models) 
    m->modes(means(i), var(i), x_s);

  // precompute likelihoods (for they don't change in y)
  arr log_likelihood(models.N);
  arr likelihood(models.N);
	i=0;
  for_list(i, m, models) {
    log_likelihood(i) = m->log_likelihood();  

    // this is unfortunately numerical unstable, but I don't know how to fix it
    likelihood(i) = exp(log_likelihood(i)); 
  }

  double sum_dkl=0; 
  i=0;
  for_list(i, m, models) { 
    arr sigma1 = ARR(var(i));
    sigma1.reshape(1,1);
    for_array(j, mm, models) {
      arr sigma2 = ARR(var(j));
      sigma2.reshape(1,1);
      sum_dkl += kullback_leibler_divergence(ARR(means(i)), sigma1, ARR(means(j)), sigma2);
    }
  }
  return -sum_dkl; 
}
