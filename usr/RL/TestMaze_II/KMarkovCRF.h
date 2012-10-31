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
        virtual double evaluate(DataGiven data_given, DataPredict data_predict) {
            double ret = 0;
            ret += std::get<0>(data_given);
            ret += data_predict;
            return ret;
        }
    };

    template < class DataGiven, class DataPredict >
    class Factor {
    public:
        double evaluate(DataGiven data_given, DataPredict data_predict) {
            double sum = 0;
            for(unsigned int k_idx=0; k_idx<features.size(); ++k_idx) {
                sum += lambda[k_idx]*features[k_idx].evaluate(data_given,data_predict);
            }
            return exp(sum);
        }
        std::vector<Feature<DataGiven,DataPredict> > features;
        lbfgsfloatval_t * lambda;
    };

    KMarkovCRF(const int& kk, const int& dim_x, const int& dim_y, const int& action_n);

    virtual ~KMarkovCRF();

    static lbfgsfloatval_t static_evaluate(
            void *instance,
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n,
            const lbfgsfloatval_t step
    );

    lbfgsfloatval_t evaluate(
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n
    );

    static int static_progress(
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

    int progress(
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

    int optimize();

    void add_action_state_reward_tripel(
            const int& state,
            const int& action,
            const double& reward
    );

    void check_derivative(const int& number_of_samples, const double& range = 1, const double& max_difference = 1e-6 );

private:
    int k, lambda_size;;
    double z;
    bool z_up_to_date;
    episode_t episode_data;
    lbfgsfloatval_t * lambda;
    std::vector<Factor<std::tuple<int,int>, int> > state_factors;

    class SimpleMDPFeature: public Feature<std::tuple<int,int>,int> {
    public:
        SimpleMDPFeature(const int& s_from, const int& a, const int& s_to):
            state_from(s_from), action(a), state_to(s_to) {}
        virtual double evaluate(std::tuple<int,int> data_given, int data_predict) {
            if( std::get<0>(data_given)==state_from && std::get<1>(data_given)==action && data_predict==state_to) {
                return 1;
            } else {
                return 0;
            }
        }
    private:
        int state_from, action, state_to;
    };

    double update_z();

    double raw_state_probability(const int& state_from, const int& action, const int& state_to);
};

#endif /* KMARKOVCRF_H_ */
