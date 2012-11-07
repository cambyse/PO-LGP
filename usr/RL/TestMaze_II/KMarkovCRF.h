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
#include <memory>

#include <QtCore/QString>

#include <lbfgs.h>

#include "debug.h"

class KMarkovCRF
{
public:

    typedef int state_t;
    typedef int action_t;
    typedef double reward_t;
    typedef double probability_t;
    typedef std::tuple<action_t,state_t,reward_t> data_point_t;
    typedef std::vector<data_point_t> episode_t;
    typedef episode_t::const_iterator const_episode_iterator_t;


    KMarkovCRF(const int& k_state_, const int& k_reward_, const int& k, const int& x, const int& y, const int& a_n);

    virtual ~KMarkovCRF();

    static lbfgsfloatval_t static_evaluate_model(
            void *instance,
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n,
            const lbfgsfloatval_t step
    );

    lbfgsfloatval_t evaluate_model(
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n
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

    void optimize_model();

    void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );

    void clear_data() { episode_data.clear(); }

    void check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);

private:

    template < class Data, class DataPredict>
    class Feature {
    protected:
        typedef Data D;
        typedef DataPredict DP;
        typedef std::vector<Feature<Data,DataPredict> * > subfeature_container_t;
    public:
        virtual double evaluate(Data) const = 0;
        virtual double evaluate(Data, DataPredict) const = 0;
        virtual std::string identifier() const = 0;
        subfeature_container_t subfeatures;
    protected:
        static int field_width[2];
    };
    typedef Feature<const_episode_iterator_t, const data_point_t> feature_t;

    class ActionFeature: public feature_t {
    public:
        ActionFeature(const action_t& a, const int& d):
            action(a), delay(d) {
            if(field_width[0]<5) field_width[0]=5;
            if(field_width[1]<log10(delay)) field_width[1]=log10(delay);
            subfeatures.push_back(this);
        }
        virtual double evaluate(D data) const { return std::get<0>(*(data+=delay))==action ? 1 : 0; }
        virtual double evaluate(D data, DP) const {
            return evaluate(data);
        }
        virtual std::string identifier() const { return ("a("+QString(5-field_width[0],' ')+QString(action_strings[action])+", "+QString("%1").arg(delay,field_width[1])+")").toStdString(); }
    private:
        action_t action;
        int delay;
        static const char* action_strings[5];
    };

    class StateFeature: public feature_t {
    public:
        StateFeature(const state_t& s, const int& d):
            state(s), delay(d) {
            if(field_width[0]<log10(state)) field_width[0]=log10(state);
            if(field_width[1]<log10(delay)) field_width[1]=log10(delay);
            subfeatures.push_back(this);
        }
        virtual double evaluate(D data) const { return std::get<1>(*(data+=delay))==state ? 1 : 0; }
        virtual double evaluate(D data, DP data_predict) const { return delay==0 ? ( std::get<1>(data_predict)==state ? 1 : 0 ) : ( std::get<1>(*(data+=delay))==state ? 1 : 0 ) ; }
        virtual std::string identifier() const { return ("s("+QString("%1").arg(state,field_width[0])+", "+QString("%1").arg(delay,field_width[1])+")").toStdString(); }
    private:
        state_t state;
        int delay;
    };

    class RewardFeature: public feature_t {
    public:
        RewardFeature(const reward_t& r, const int& d):
            reward(r), delay(d) {
            if(field_width[0]<2) field_width[0]=2;
            if(field_width[1]<log10(delay)) field_width[1]=log10(delay);
            subfeatures.push_back(this);
        }
        virtual double evaluate(D data) const { return std::get<2>(*(data+=delay))==reward ? 1 : 0; }
        virtual double evaluate(D data, DP data_predict) const { return delay==0 ? ( std::get<2>(data_predict)==reward ? 1 : 0 ) : ( std::get<2>(*(data+=delay))==reward ? 1 : 0 ) ; }
        virtual std::string identifier() const { return ("r("+QString("%1").arg(reward,field_width[0])+", "+QString("%1").arg(delay,field_width[1])+")").toStdString(); }
    private:
        reward_t reward;
        int delay;
    };

    class AndFeature: public feature_t {
    public:
        AndFeature(const feature_t& f1, const feature_t& f2) {
            subfeatures.insert(subfeatures.begin(),f1.subfeatures.begin(),f1.subfeatures.end());
            subfeatures.insert(subfeatures.begin(),f2.subfeatures.begin(),f2.subfeatures.end());
        }
        virtual double evaluate(D data) const {
            double prod = 1;
            for(subfeature_container_t::const_iterator feature_iterator=subfeatures.begin();
                    feature_iterator!=subfeatures.end();
                    ++feature_iterator) {
                prod *= (*feature_iterator)->evaluate(data);
            }
            return prod;
        }
        virtual double evaluate(D data, DP data_predict) const {
            double prod = 1;
            for(subfeature_container_t::const_iterator feature_iterator=subfeatures.begin();
                    feature_iterator!=subfeatures.end();
                    ++feature_iterator) {
                prod *= (*feature_iterator)->evaluate(data, data_predict);
            }
            return prod;
        }
        virtual std::string identifier() const {
            std::string id;
            bool first = true;
            for(subfeature_container_t::const_iterator feature_iterator=subfeatures.begin();
                    feature_iterator!=subfeatures.end();
                    ++feature_iterator) {
                if(!first) id += "<and>";
                id += (*feature_iterator)->identifier();
                first = false;
            }
            return id;
        };
    };

    int k, number_of_features, x_dim, y_dim, action_n;
    episode_t episode_data;
    lbfgsfloatval_t * lambda;
    std::vector<int> parameter_indices;
    std::vector<std::unique_ptr<feature_t> > features;

};

#include "debug_exclude.h"

#endif /* KMARKOVCRF_H_ */
