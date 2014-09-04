#ifndef TEMPORALLYEXTENDEDFEATURELEARNER_H_
#define TEMPORALLYEXTENDEDFEATURELEARNER_H_

#include "../HistoryObserver.h"
#include "../SpaceManager.h"
#include "../optimization/LBFGS_Object.h"

#include <memory>
#include <vector>
#include <map>

//#define ARMA_NO_DEBUG
#include <armadillo>

class ConjunctiveAdjacency;

class TemporallyExtendedFeatureLearner: public HistoryObserver, public virtual SpaceManager {

    //----typedefs/classes----//
public:
    DISAMBIGUATE_CONFIG_TYPEDEFS(HistoryObserver);
    typedef arma::SpMat<double> f_mat_t; // using double for compatibility to vec_t
    typedef arma::Col<double> col_vec_t;
    typedef arma::Row<double> row_vec_t;
    typedef std::map<f_ptr_t,double> weight_map_t;
    typedef Feature::look_up_map_t basis_feature_map_t;
    enum class OUTCOME_TYPE { ACTION, OBSERVATION_REWARD };

    //----members----//

private:

    /**  Whether "outcome" means "action" (value based) or "observation-reward"
     * (model based). This affects the dimensionality of the F-matrices. Use
     * TemporallyExtendedFeatureLearner::set_outcome_type to change. */
    OUTCOME_TYPE outcome_type = OUTCOME_TYPE::OBSERVATION_REWARD;

protected:

    /** The set of features used by the method. */
    f_set_t feature_set;

    /** Adjacency operator used to grow the feature set. */
    std::shared_ptr<ConjunctiveAdjacency> N_plus;

    /**Contains for every data point a matrix with all feature values for all
     * possible outcomes. \f$F_{ij}\f$ is the i-th feature for the j-th
     * outcome. To be updated on changes of the data or the feature set.
     *
     * If outcome_type is OBSERVATION_REWARD (model based), the matrix with
     * (time) index t holds features evaluated on
     * \f$\{\ldots,(a_{t-1},o_{t-1},r_{t-1}),(a_{t},O,R)\}\f$ where O and R are
     * the outcome.
     *
     * If outcome_type is ACTION (value based), the matrix with (time) index t
     * holds features evaluated on
     * \f$\{\ldots,(a_{t},o_{t},r_{t}),(A,O^*,R^*)\}\f$ where A is the outcome
     * and O* and R* are the default elements of the observation and reward
     * space. A value based method should ensure that featues don't depend on
     * observations or rewards with time index of zero since these are
     * considered to represent the immediate future. */
    std::vector<f_mat_t> F_matrices;

    /** Number of outcomes (action or observation-reward pairs). */
    int outcome_n = 0;

    /** Contains for every data point the index of the column in of the F-matrix
     * that was actually observed. To be updated on changes of the data. */
    std::vector<int> outcome_indices;

    /** Weight for the features. */
    col_vec_t weights;

    /** Whether the training data changed. */
    bool data_changed = true;

    /** Whether the features changed. */
    bool feature_set_changed = true;

    /** Whether the basis features changed. */
    bool basis_features_changed = true;

    /** The coefficient for L1-regularization. */
    double l1_factor = 0;

    /** Set of basis features.
     *
     * To be updated on changes of the feature set. */
    f_ptr_set_t basis_features;

    /** Contains for all data points and all possible outcomes a map with values
     * of all basis features (to compute the F-matrices more efficiently). To be
     * updated on changes of the basis feature set or the data. */
    std::vector<std::vector<basis_feature_map_t> > basis_feature_maps;

    /** Counts evaluations of the objective. Variable is reset when optization
     * is triggered to report the overall number of objective evaluations
     * (including e.g. linesearch).*/
    int objective_evaluations = 0;

    //----methods----//

public:
    TemporallyExtendedFeatureLearner(std::shared_ptr<ConjunctiveAdjacency> N);
    virtual ~TemporallyExtendedFeatureLearner() = default;
    virtual void grow_feature_set();
    virtual void shrink_feature_set();
    virtual f_set_t get_feature_set();
    virtual void set_feature_set(const f_set_t&);
    virtual void set_l1_factor(const double& l1);
    virtual void print_features() const;
    virtual void print_training_data() const;
    virtual void add_action_observation_reward_tripel(
        const action_ptr_t& action,
        const observation_ptr_t& observation,
        const reward_ptr_t& reward,
        const bool& new_episode
        ) override;
    virtual double optimize_weights_LBFGS();
    /** Directly uses LBFGS_Object::check_derivatives (see there for docu). */
    bool check_derivatives(const int& number_of_samples,
                           const double& range,
                           const double& delta = 1e-5,
                           const double& allowed_maximum_relative_deviation = 1e-5,
                           const double& minimum_gradient = 0,
                           const bool use_current_values = false
        );
    virtual void clear_data() override;
    virtual void set_spaces(const action_ptr_t & a,
                            const observation_ptr_t & o,
                            const reward_ptr_t & r) override;
    /** Print feature matrices. That is, print values of all feature for all
     * outcomes for all training data. @param n gives the number of previous
     * action-observation-reward triplets to be printed. */
    virtual void print_F_matrices(int n = 0);

protected:

    /** Set the outcome type. */
    void set_outcome_type(OUTCOME_TYPE);

    /** Update number of outcomes. */
    void update_outcome_n();

    virtual weight_map_t get_weight_map() const;
    virtual void apply_weight_map(weight_map_t);

    /** Update all data. */
    virtual bool update();

    /** Update basis features from current feature set and returns whether they
     * changed. */
    virtual bool update_basis_features();

    /** Update maps for basis features (needs up-to-date basis features).
     * @param recompute_all Whether to recompute values for all features (if
     * e.g. data changed) or only for the missing ones. */
    virtual void update_basis_feature_maps(bool recompute_all);

    /** Updates F-matrices for feature set (needs up-to-date basis feature
     * maps). */
    virtual void update_F_matrices();

    /** Update outcome indices. */
    virtual void update_outcome_indices();

    /** Pick only features that are non-constant for all data points and
     * outcomes. Caution: For efficiency basis_feature_maps are used and must be
     * up to date! */
    virtual bool pick_non_const_features();
    virtual lbfgsfloatval_t LBFGS_objective(const lbfgsfloatval_t*, lbfgsfloatval_t*) = 0;
    virtual int LBFGS_progress(const lbfgsfloatval_t *x,
                               const lbfgsfloatval_t *g,
                               const lbfgsfloatval_t fx,
                               const lbfgsfloatval_t xnorm,
                               const lbfgsfloatval_t gnorm,
                               const lbfgsfloatval_t step,
                               int nr_variables,
                               int iteration_nr,
                               int ls) const = 0;
    virtual void LBFGS_final_message(double) const = 0;
    virtual LBFGS_Object::objective_t get_LBFGS_objective();
    virtual LBFGS_Object::progress_t get_LBFGS_progress() const;
};

#endif /* TEMPORALLYEXTENDEDFEATURELEARNER_H_ */
