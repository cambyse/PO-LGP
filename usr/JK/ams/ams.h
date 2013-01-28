#ifndef _HEADER_GUARD_AMS_H_
#define _HEADER_GUARD_AMS_H_

#include <MT/array.h>

/**
 * References:
 * [1] J. Kulick, R. Lieck, M. Toussaint, "Active Model Selection by Minimizing
 *     the Expected Entropy", 2013, ICML 
 **/

/**
 * ams_model is the interface for active model selection. It is the more or less
 * the domain of the random variable M in [1]
 * */
class ams_model {
  public:
    virtual const double log_likelihood() const = 0;
    virtual const double prediction(const arr& x, const double y) const = 0;
    virtual void add_observation(const arr& x, const double y) = 0;
    virtual void modes(double& mean, double& var, const arr& x) = 0;
    virtual ams_model* clone() const = 0;
    virtual ~ams_model() {};
};

typedef MT::Array<ams_model*> model_list_t;

/**
 * This is the literal translation of the formulas in [1]. All integrals are
 * done as Euler integrations. The result is only proportional to the actual
 * ecpected entropy, since we only need its extrema.
 *
 * @param models The discrete set of models
 * @param x_s x*
 * @result The expected entropy as described in [1]
 * */
double expected_entropy(model_list_t& models, const arr& x_s);

double exp_entropy_learned(model_list_t& models, const arr& x_s);

#endif // _HEADER_GUARD_AMS_H_


