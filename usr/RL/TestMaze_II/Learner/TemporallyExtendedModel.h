#ifndef TEMPORALLYEXTENDEDMODEL_H_
#define TEMPORALLYEXTENDEDMODEL_H_

#include "../HistoryObserver.h"
#include "../Predictor.h"
#include "../Representation/Feature.h"
#include "../optimization/LBFGS_Object.h"

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
    /** \brief The set of features used by the model. */
    f_set_t feature_set;

    /** \brief Adjacency operator used to grow the feature set. */
    std::shared_ptr<ConjunctiveAdjacency> N_plus;

    /** \brief Contains for every data point a matrix with all feature values
     * for all possible outcomes. */
    std::vector<f_mat_t> F_matrices;

    /** \brief Contains for every data point the index of the column in of the
     * F-matrix that was actually observed. */
    std::vector<int> outcome_indices;

    /** \brief Weight for the features. */
    vec_t weights;

    /** \brief Whether the training data changed. */
    bool data_changed = true;

    /** \brief Whether the features changed. */
    bool features_changed = true;

    /** \brief The coefficient for L1-regularization. */
    double l1_factor = 0;

    /** \brief Contains for all data points and all possible outcomes a map with
     * values of all basis features (to compute the F-matrices more
     * efficiently). */
    std::vector<std::vector<f_ptr_set_t> > basis_feature_map;

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
    /* virtual void optimize_weights_SGD(); */
    virtual void optimize_weights_LBFGS();
    virtual void grow_feature_set();
    virtual void shrink_feature_set();
    virtual void set_feature_set(const f_set_t&);
    virtual void set_l1_factor(const double& l1);
    virtual void print_features() const;
    virtual void print_training_data() const;
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
    virtual void update();
    virtual double neg_log_likelihood(vec_t& grad, const vec_t& weights);
    lbfgsfloatval_t LBFGS_objective(const lbfgsfloatval_t*, lbfgsfloatval_t*);
    int LBFGS_progress(const lbfgsfloatval_t *x,
                       const lbfgsfloatval_t *g,
                       const lbfgsfloatval_t fx,
                       const lbfgsfloatval_t xnorm,
                       const lbfgsfloatval_t gnorm,
                       const lbfgsfloatval_t step,
                       int nr_variables,
                       int iteration_nr,
                       int ls) const;
    virtual LBFGS_Object::objective_t get_LBFGS_objective();
    virtual LBFGS_Object::progress_t get_LBFGS_progress() const;
};

#endif /* TEMPORALLYEXTENDEDMODEL_H_ */
