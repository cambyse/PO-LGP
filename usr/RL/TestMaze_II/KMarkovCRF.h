/*
 * KMarkovCRF.h
 *
 *  Created on: Oct 30, 2012
 *      Author: robert
 */

#ifndef KMARKOVCRF_H_
#define KMARKOVCRF_H_

#include <vector>
#include <tuple>
#include <math.h>

#include <lbfgs.h>

class KMarkovCRF
{
public:

    typedef std::vector<std::tuple<int,int,double> > episode_t;
    typedef episode_t::const_iterator data_point_t;

    template < class DataGiven, class DataPredict >
    class Feature {
    public:
        virtual double evaluate(DataGiven data_given, DataPredict data_predict) = 0;
    };

    KMarkovCRF(const int& k_state_, const int& k_reward_, const int& x, const int& y, const int& a_n);

    virtual ~KMarkovCRF();

    static lbfgsfloatval_t static_evaluate_state_model(
            void *instance,
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n,
            const lbfgsfloatval_t step
    );

    static lbfgsfloatval_t static_evaluate_reward_model(
            void *instance,
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n,
            const lbfgsfloatval_t step
    );

    lbfgsfloatval_t evaluate_state_model(
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n
    );

    lbfgsfloatval_t evaluate_reward_model(
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n
    );

    static int static_progress_state_model(
            void *instance,
            const lbfgsfloatval_t *x,
            const lbfgsfloatval_t *g,
            const lbfgsfloatval_t fx,
            const lbfgsfloatval_t xnorm,
            const lbfgsfloatval_t gnorm,
            const lbfgsfloatval_t step,
            int n,
            int k,
            int ls
    );

    static int static_progress_reward_model(
            void *instance,
            const lbfgsfloatval_t *x,
            const lbfgsfloatval_t *g,
            const lbfgsfloatval_t fx,
            const lbfgsfloatval_t xnorm,
            const lbfgsfloatval_t gnorm,
            const lbfgsfloatval_t step,
            int n,
            int k,
            int ls
    );

    int progress_state_model(
            const lbfgsfloatval_t *x,
            const lbfgsfloatval_t *g,
            const lbfgsfloatval_t fx,
            const lbfgsfloatval_t xnorm,
            const lbfgsfloatval_t gnorm,
            const lbfgsfloatval_t step,
            int n,
            int k,
            int ls
    );

    int progress_reward_model(
            const lbfgsfloatval_t *x,
            const lbfgsfloatval_t *g,
            const lbfgsfloatval_t fx,
            const lbfgsfloatval_t xnorm,
            const lbfgsfloatval_t gnorm,
            const lbfgsfloatval_t step,
            int n,
            int k,
            int ls
    );

    void optimize_state_model();

    void optimize_reward_model();

    void add_action_state_reward_tripel(
            const int& action,
            const int& state,
            const double& reward
    );

    void clear_data() { episode_data.clear(); }

    void check_state_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);
    void check_reward_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);

    double state_probability(const int& state_from, const int& action, const int& state_to, lbfgsfloatval_t const * x = nullptr);
    double reward_probability(const std::vector<int> & state_vector, const double& rew, lbfgsfloatval_t const * x = nullptr);

private:

    class MDPFeature: public Feature<std::tuple<int,int>,int> {
    public:
        MDPFeature(const int& s_from, const int& a, const int& s_to):
            state_from(s_from), action(a), state_to(s_to) {}
        virtual double evaluate(std::tuple<int,int> data_given, int data_predict) {
            if( std::get<0>(data_given)==state_from && std::get<1>(data_given)==action && data_predict==state_to) {
                return 1;
            } else {
                return 0;
            }
        }
        int get_state_from() const { return state_from; }
        int get_action()     const { return action;     }
        int get_state_to()   const { return state_to;   }
    private:
        int state_from, action, state_to;
    };

    class RewardFeature: public Feature<std::vector<int>,double> {
    public:
        RewardFeature(const int& s, const int& d):
            state(s), delay(d) {}
        virtual double evaluate(std::vector<int> data_given, double data_predict) {
            if( data_given[delay]==state && data_predict>0) {
                return 1;
            } else {
                return 0;
            }
        }
        int get_state() const { return state; }
        int get_delay() const { return delay; }
    private:
        int state, delay;
    };

    int k_state, k_reward, lambda_state_size, lambda_reward_size, x_dim, y_dim, action_n;
    episode_t episode_data;
    lbfgsfloatval_t * lambda_state;
    lbfgsfloatval_t * lambda_reward;
    std::vector<int> state_parameter_indices;
    std::vector<int> reward_parameter_indices;
    std::vector<MDPFeature> state_features;
    std::vector<RewardFeature> reward_features;

    double raw_state_probability(const int& state_from, const int& action, const int& state_to, lbfgsfloatval_t const * x = nullptr);
    double state_partition_function(const int& state_from, const int& action, lbfgsfloatval_t const * x = nullptr);
    double raw_reward_probability(const std::vector<int> & state_vector, const double& rew, lbfgsfloatval_t const * x = nullptr);
    double reward_partition_function(const std::vector<int> & state_vector, lbfgsfloatval_t const * x = nullptr);

};

#endif /* KMARKOVCRF_H_ */
