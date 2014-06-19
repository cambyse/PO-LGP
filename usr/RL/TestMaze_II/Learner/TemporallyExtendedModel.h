#ifndef TEMPORALLYEXTENDEDMODEL_H_
#define TEMPORALLYEXTENDEDMODEL_H_

#include "../Predictor.h"
#include "TemporallyExtendedFeatureLearner.h"

class TemporallyExtendedModel: public Predictor, public TemporallyExtendedFeatureLearner {

    //---- typedefs ----//

public:

    DISAMBIGUATE_CONFIG_TYPEDEFS(TemporallyExtendedFeatureLearner);

    //---- members ----//

    using TemporallyExtendedFeatureLearner::action_space;
    using TemporallyExtendedFeatureLearner::observation_space;
    using TemporallyExtendedFeatureLearner::reward_space;

    //---- methods ----//

public:

    TemporallyExtendedModel(std::shared_ptr<ConjunctiveAdjacency>);
    virtual ~TemporallyExtendedModel() = default;
    virtual probability_t get_prediction(const_instance_ptr_t,
                                         const action_ptr_t&,
                                         const observation_ptr_t&,
                                         const reward_ptr_t&) const override;
    virtual probability_map_t get_prediction_map(const_instance_ptr_t,
                                                 const action_ptr_t&) const override;

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

#endif /* TEMPORALLYEXTENDEDMODEL_H_ */
