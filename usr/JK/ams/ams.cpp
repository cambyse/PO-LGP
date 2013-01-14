#include "ams.h"

#include <MT/array_t.cxx>
#include <JK/util.h>
#include <cmath>
#include <devTools/logging.h>

SET_LOG(ams, DEBUG);

double expected_entropy_on_y(model_list_t& models, const arr& x_s, const double yi, const arr& log_likelihood, const arr& likelihood) {
  double exp_entropy = 0;
  double mixture = 10e-10; //should be zero, but for numeric reasons we lift it a little bit
  arr p_y_given_xM(models.N);
 
  //compute the mixture 
  uint i; ams_model *model;
  for_list(i, model, models) {
    p_y_given_xM(i) = model->prediction(x_s, yi); // cache the conditioned probability
    mixture += (likelihood(i) * p_y_given_xM(i));
  }

  uint m; 
  for_list(m, model, models) {
    exp_entropy += likelihood(m) * p_y_given_xM(m) * (log_likelihood(m) + log(p_y_given_xM(m)) - log(mixture));
  }
  return exp_entropy;  
}

double expected_entropy(model_list_t& models, const arr& x_s) {
  //First try: Euler integration
  double eps = 1./500.;
  double exp_entropy = 0;

  // precompute likelihoods (for they don't change in y)
  arr log_likelihood(models.N);
  arr likelihood(models.N);
  for_array(i, m, models) {
    log_likelihood(i) = m->log_likelihood();  

    // this is unfortunately numerical unstable, but I don't know how to fix it
    likelihood(i) = exp(log_likelihood(i)); 
  }

  for_range(yi, -1000, 1000) {
    // should actually be += eps * ..., but not needed for optimization
    exp_entropy += expected_entropy_on_y(models, x_s, yi*eps, log_likelihood, likelihood); 
  }

  // again this should be -eta*exp_entropy but for optimization we don't care
  return -exp_entropy; 
}

double exp_entropy_learned(model_list_t& models, const arr& x_s) {
  double eps = 1./500.;
  double exp_entropy;

  for_range(yi, -1000, 1000) {
    model_list_t learned;
    
    double mixture = 0;
    arr likelihood(models.N);
    arr log_likelihood(models.N);

    uint m; ams_model* model;
    for_list(m, model, models) {
      ams_model* learned_model = model->clone();
      learned_model->add_observation(x_s, yi*eps);
      log_likelihood(m) = learned_model->log_likelihood();
      delete learned_model;
      likelihood(m) = exp(log_likelihood(m));
      mixture += likelihood(m);
    }
    if(mixture < 1e-40) mixture=1e-40;

    for_upper(m, models.N) {
      exp_entropy += likelihood(m) * (log_likelihood(m) - log(mixture));
    }
  }  
  return -exp_entropy;
}

