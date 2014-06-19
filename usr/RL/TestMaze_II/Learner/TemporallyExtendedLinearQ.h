#ifndef TEMPORALLYEXTENDEDLINEARQ_H_
#define TEMPORALLYEXTENDEDLINEARQ_H_

#include "../Planning/Policy.h"
#include "TemporallyExtendedFeatureLearner.h"

class TemporallyExtendedLinearQ: public Policy, public TemporallyExtendedFeatureLearner {

    //----typedefs/classes----//

public:

    DISAMBIGUATE_CONFIG_TYPEDEFS(TemporallyExtendedFeatureLearner);

    //----members----//

    //----methods----//

public:

    TemporallyExtendedLinearQ(std::shared_ptr<ConjunctiveAdjacency>);

    virtual ~TemporallyExtendedLinearQ() = default;

    virtual action_ptr_t get_action(const_instance_ptr_t) override;

protected:

    virtual double neg_log_likelihood(col_vec_t& grad, const col_vec_t& weights);
    lbfgsfloatval_t LBFGS_objective(const lbfgsfloatval_t*, lbfgsfloatval_t*) override;
    int LBFGS_progress(const lbfgsfloatval_t *x,
                       const lbfgsfloatval_t *g,
                       const lbfgsfloatval_t fx,
                       const lbfgsfloatval_t xnorm,
                       const lbfgsfloatval_t gnorm,
                       const lbfgsfloatval_t step,
                       int nr_variables,
                       int iteration_nr,
                       int ls) const override;

};

#endif /* TEMPORALLYEXTENDEDLINEARQ_H_ */
