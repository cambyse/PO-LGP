#ifndef TEMPORALLYEXTENDEDMODEL_H_
#define TEMPORALLYEXTENDEDMODEL_H_

#include "../HistoryObserver.h"
#include "../Predictor.h"
#include "../Representation/Feature.h"

#include <lbfgs.h>
#include <memory>
#include <vector>
#include <map>

//#define ARMA_NO_DEBUG
#include <armadillo>

class ConjunctiveAdjacency;

class TemporallyExtendedModel: public HistoryObserver, public Predictor {
    //---- typedefs ----//
public:
    DISAMBIGUATE_CONFIG_TYPEDEFS(HistoryObserver);
    typedef arma::SpMat<double> f_mat_t; // using double for compatibility to vec_t
    typedef arma::Col<double> vec_t;
    typedef std::map<f_ptr_t,double> weight_map_t;
    //---- members ----//
private:
    feature_set_t feature_set;
    std::shared_ptr<ConjunctiveAdjacency> N_plus;
    std::vector<f_mat_t> F_matrices;
    std::vector<int> outcome_indices;
    vec_t weights;
    bool data_up_to_date = false;
    //---- methods ----//
public:
    TemporallyExtendedModel(std::shared_ptr<ConjunctiveAdjacency>);
    virtual ~TemporallyExtendedModel() = default;
    virtual probability_t get_prediction(const_instance_ptr_t,
                                         const action_ptr_t&,
                                         const observation_ptr_t&,
                                         const reward_ptr_t&) const;
    virtual void add_action_observation_reward_tripel(
        const action_ptr_t& action,
        const observation_ptr_t& observation,
        const reward_ptr_t& reward,
        const bool& new_episode
        ) override;
    virtual void optimize_weights();
    virtual void grow_feature_set();
    virtual void shrink_feature_set();
    virtual void set_feature_set(const feature_set_t&);
    virtual void print_features() const;
    virtual void print_training_data() const;
    lbfgsfloatval_t LBFGS_objective(const lbfgsfloatval_t*, lbfgsfloatval_t*);
    /** \brief Directly uses LBFGS_Object::check_derivatives (see there for
     * docu). */
    bool check_derivatives(const int& number_of_samples,
                           const double& range,
                           const double& delta = 1e-5,
                           const double& allowed_maximum_relative_deviation = 1e-5,
                           const double& minimum_gradient = 0,
                           const bool use_current_values = false
        );
protected:
    virtual weight_map_t get_weight_map() const;
    virtual void apply_weight_map(weight_map_t);
    virtual void update_data();
    virtual double log_likelihood(vec_t& grad, const vec_t& weights);
    virtual double log_likelihood(vec_t& grad);
};

#endif /* TEMPORALLYEXTENDEDMODEL_H_ */
