#ifndef _HEADER_GUARD_GP_AMS_H_
#define _HEADER_GUARD_GP_AMS_H_

#include "ams.h"
#include <Core/array.h>

/**
 * gp_ams_model implementation for classification problems.
 * */
class gp_classification : public ams_model {
  public:
		class GPC* gp;
    gp_classification(const gp_classification& r);
    gp_classification(GPC* gpc);
    ~gp_classification();
    virtual const double log_likelihood() const;
    virtual const double prediction(const arr& x, const double y) const;
    virtual void add_observation(const arr& x, const double y);
    virtual void modes(double& mean, double& var, const arr& x);
    virtual ams_model* clone() const;
};

/**
 * gp_ams_model implementation for regression problems.
 * */
class gp_regression : public ams_model {
  public:
	  class GaussianProcess* gp;
    gp_regression(const gp_regression& r);
    gp_regression(GaussianProcess* gpr);
    ~gp_regression();
    virtual const double log_likelihood() const;
    virtual const double prediction(const arr& x, const double y) const;
    virtual void add_observation(const arr& x, const double y);
    virtual void modes(double& mean, double& var, const arr& x);
    virtual ams_model* clone() const;
};

/**
 * This is an efficient approximation of the the actual expected entropy, which
 * computes the sum of Kullback-Leibler-Divergences of the current gaussian
 * process model and each gaussian of the mixture.
 * @param models The discrete set of models
 * @param x_s x*
 * @result The sum of DKLs as described in [1]
 * */
double sum_of_dkls(model_list_t& models, const arr& x_s);

#endif // _HEADER_GUARD_GP_AMS_H_


