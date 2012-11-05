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
#include <string>

#include <QtCore/QString>

#include <lbfgs.h>

class KMarkovCRF
{
public:

    typedef int state_t;
    typedef int action_t;
    typedef double reward_t;
    typedef double probability_t;
    typedef std::tuple<action_t,state_t,reward_t> data_point_t;
    typedef std::vector<data_point_t> episode_t;


    KMarkovCRF(const int& k_state_, const int& k_reward_, const int& k, const int& x, const int& y, const int& a_n);

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

    static lbfgsfloatval_t static_evaluate_model(
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

    lbfgsfloatval_t evaluate_model(
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

    static int static_progress_model(
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

    int progress_model(
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

    void optimize_model();

    void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );

    void clear_data() { episode_data.clear(); }

    void check_state_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);
    void check_reward_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);
    void check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);

    probability_t state_probability(const state_t& state_from, const action_t& action, const state_t& state_to, lbfgsfloatval_t const * x = nullptr);
    probability_t reward_probability(const std::vector<state_t> & state_vector, const reward_t& rew, lbfgsfloatval_t const * x = nullptr);
    probability_t probability(const episode_t& episode, lbfgsfloatval_t const * x = nullptr);

private:

    static const char* action_strings[5];

    template < class Data>
    class Feature {
    public:
        virtual double evaluate(const Data& data) const {
        Data d = data; // todo aaarggg....
        return double();
    }
        virtual std::string identifier() { return QString().toStdString(); }
    };

    class ActionFeature: public Feature<episode_t> {
    public:
        ActionFeature(const action_t& a, const int& d):
            action(a), delay(d) {}
        virtual double evaluate(const episode_t& data_given) const {
            if( std::get<0>(data_given[delay])==action) {
                return 1;
            } else {
                return 0;
            }
        }
        virtual std::string identifier() { return ("("+QString(action_strings[action])+","+QString::number(delay)+")").toStdString(); }
    private:
        action_t action;
        int delay;
    };

    class StateFeature: public Feature<episode_t> {
    public:
        StateFeature(const state_t& s, const int& d):
            state(s), delay(d) {}
        virtual double evaluate(const episode_t& data_given) const {
            if( std::get<1>(data_given[delay])==state) {
                return 1;
            } else {
                return 0;
            }
        }
        virtual std::string identifier() { return ("("+QString::number(state)+","+QString::number(delay)+")").toStdString(); }
    private:
        state_t state;
        int delay;
    };

    class RewardFeature: public Feature<episode_t> {
    public:
        RewardFeature(const reward_t& r, const int& d):
            reward(r), delay(d) {}
        virtual double evaluate(const episode_t& data_given) const {
            if( std::get<2>(data_given[delay])==reward) {
                return 1;
            } else {
                return 0;
            }
        }
        virtual std::string identifier() { return ("("+QString::number(reward,'g',1)+","+QString::number(delay)+")").toStdString(); }
    private:
        reward_t reward;
        int delay;
    };

    template < class DataGiven, class DataPredict >
    class OldFeature {
    public:
        virtual double evaluate(const DataGiven& data_given, const DataPredict& data_predict) = 0;
    };

    class MDPFeature: public OldFeature<std::tuple<int,int>,int> {
    public:
        MDPFeature(const int& s_from, const int& a, const int& s_to):
            state_from(s_from), action(a), state_to(s_to) {}
        virtual double evaluate(const std::tuple<int,int>& data_given, const int& data_predict) {
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

    class OldRewardFeature: public OldFeature<std::vector<int>,reward_t> {
    public:
        OldRewardFeature(const int& s, const int& d):
            state(s), delay(d) {}
        virtual double evaluate(const std::vector<int>& data_given, const reward_t& data_predict) {
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

    int k_state, k_reward, k, lambda_state_size, lambda_reward_size, lambda_size, x_dim, y_dim, action_n;
    episode_t episode_data;
    lbfgsfloatval_t * lambda_state;
    lbfgsfloatval_t * lambda_reward;
    lbfgsfloatval_t * lambda;
    std::vector<int> state_parameter_indices;
    std::vector<int> reward_parameter_indices;
    std::vector<int> parameter_indices;
    std::vector<MDPFeature> state_features;
    std::vector<OldRewardFeature> reward_features;
    std::vector<Feature<episode_t> > features;

    probability_t raw_state_probability(const int& state_from, const int& action, const int& state_to, lbfgsfloatval_t const * x = nullptr);
    probability_t state_partition_function(const int& state_from, const int& action, lbfgsfloatval_t const * x = nullptr);
    probability_t raw_reward_probability(const std::vector<int> & state_vector, const reward_t& rew, lbfgsfloatval_t const * x = nullptr);
    probability_t reward_partition_function(const std::vector<int> & state_vector, lbfgsfloatval_t const * x = nullptr);
    probability_t raw_probability(const episode_t& episode, lbfgsfloatval_t const * x = nullptr);
    probability_t partition_function(const episode_t& episode, lbfgsfloatval_t const * x = nullptr);

};

#endif /* KMARKOVCRF_H_ */
