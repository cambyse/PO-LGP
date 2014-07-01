#ifndef TEMPORALLYEXTENDEDLINEARQ_H_
#define TEMPORALLYEXTENDEDLINEARQ_H_

#include "../Planning/Policy.h"
#include "TemporallyExtendedFeatureLearner.h"

#include <tuple>

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
    bool need_to_update_c_rho_L = true;
    std::vector<action_ptr_t> policy;
    std::vector<idx_t> policy_indices;
    /** Contains a series of reward-idx pairs for computing c, rho, and L. The
     * rewards are form the data point with corresponding index. Indices are
     * serial but the first data point in each episode is skipped so that for
     * any index t t-1 is also a valid data index. */
    std::vector<std::tuple<double,int> > rewards_and_data_indices;

    //----methods----//

public:

    TemporallyExtendedLinearQ(std::shared_ptr<ConjunctiveAdjacency>,double);

    virtual ~TemporallyExtendedLinearQ() = default;

    virtual double get_action_value(const_instance_ptr_t, action_ptr_t) const;

    virtual action_ptr_t get_action(const_instance_ptr_t) override;

    virtual void set_discount(double d) { discount = d; }

    virtual double get_discount() const { return discount; }

    virtual double run_policy_iteration();

    virtual double get_TD_error();

    virtual void print_training_data() const override;

protected:
#warning should be protected (remove again!!)
public:

    virtual bool update() override;
    virtual void update_policy();
    virtual void update_rewards_and_data_indices();
    virtual action_ptr_t optimal_2x2_policy(const_instance_ptr_t) const;
    virtual void set_optimal_2x2_policy();
    virtual void update_c_rho_L();
    virtual void optimize_weights_Bellman_residual_error();
    virtual double objective_and_gradient(col_vec_t& grad, const col_vec_t& weights);

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
    virtual void LBFGS_final_message(double) const override;

};

#endif /* TEMPORALLYEXTENDEDLINEARQ_H_ */
