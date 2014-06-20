#ifndef TEMPORALLYEXTENDEDLINEARQ_H_
#define TEMPORALLYEXTENDEDLINEARQ_H_

#include "../Planning/Policy.h"
#include "TemporallyExtendedFeatureLearner.h"

class TemporallyExtendedLinearQ: public Policy, public TemporallyExtendedFeatureLearner {

    //----typedefs/classes----//

public:

    DISAMBIGUATE_CONFIG_TYPEDEFS(TemporallyExtendedFeatureLearner);

    //----members----//

private:

    double discount;
    double c;
    col_vec_t rho;
    f_mat_t L;
    std::vector<action_ptr_t> policy;
    std::vector<idx_t> policy_indices;

    //----methods----//

public:

    TemporallyExtendedLinearQ(std::shared_ptr<ConjunctiveAdjacency>,double);

    virtual ~TemporallyExtendedLinearQ() = default;

    virtual action_ptr_t get_action(const_instance_ptr_t) override;

    virtual void set_discount(double d) { discount = d; }

    virtual double get_discount() const { return discount; }

protected:

    void update_policy();
    void update_objective_components();

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
