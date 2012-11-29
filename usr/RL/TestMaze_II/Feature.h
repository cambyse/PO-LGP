/*
 * Feature.h
 *
 *  Created on: Nov 8, 2012
 *      Author: robert
 */

#ifndef FEATURE_H_
#define FEATURE_H_

#include <QString>

#include <list>
#include <set>
#include <string>
#include <memory>

#include "Data.h"

class Feature {

public:
    typedef std::list<Feature * >                  subfeature_container_t;
    typedef subfeature_container_t::iterator       subfeature_iterator_t;
    typedef subfeature_container_t::const_iterator subfeature_const_iterator_t;
    typedef std::set<std::unique_ptr<Feature> >    basis_feature_container_t;
    typedef Data::input_data_t                     input_data_t;
    typedef Data::output_data_t                    output_data_t;
    typedef Data::action_t                         action_t;
    typedef Data::state_t                          state_t;
    typedef Data::reward_t                         reward_t;
    enum TYPE { ABSTRACT, NULL_FEATURE, ACTION, STATE, REWARD, AND };

    Feature();
    virtual ~Feature();

    virtual double evaluate(input_data_t) const = 0;
    virtual double evaluate(input_data_t, output_data_t) const = 0;
    virtual std::string identifier() const;

    TYPE get_type() const;
    int get_id() const;
    unsigned int get_complexity() const;
    bool operator==(const Feature& other) const;
    bool operator!=(const Feature& other) const;
    bool operator<(const Feature& other) const;
    static bool pComp(Feature const * first, Feature const * second);

    subfeature_const_iterator_t get_subfeatures_begin() const;
    subfeature_const_iterator_t get_subfeatures_end() const;
    uint get_subfeatures_size() const;

protected:
    TYPE type;
    long id;
    unsigned int complexity;
    static int field_width[2];
    static long id_counter;
    subfeature_container_t subfeatures;
    static basis_feature_container_t basis_features;

    void clean_up_subfeatures();

};

class NullFeature: public Feature {

public:
    NullFeature();
    virtual ~NullFeature();
    virtual double evaluate(input_data_t) const;
    virtual double evaluate(input_data_t, output_data_t) const;
    virtual std::string identifier() const;
};

class ActionFeature: public Feature {

private:
    ActionFeature(const action_t& a, const int& d);
    virtual ~ActionFeature();

public:
    static ActionFeature * create(const action_t& a, const int& d);
    virtual double evaluate(input_data_t) const;
    virtual double evaluate(input_data_t, output_data_t) const;
    virtual std::string identifier() const;
private:
    action_t action;
    int delay;
};

class StateFeature: public Feature {

private:
    StateFeature(const state_t& s, const int& d);
    virtual ~StateFeature();

public:
    static StateFeature * create(const state_t& s, const int& d);
    virtual double evaluate(input_data_t) const;
    virtual double evaluate(input_data_t, output_data_t data_predict) const;
    virtual std::string identifier() const;
private:
    state_t state;
    int delay;
};

class RewardFeature: public Feature {

private:
    RewardFeature(const reward_t& r, const int& d);
    virtual ~RewardFeature();

public:
    static RewardFeature * create(const reward_t& r, const int& d);
    virtual double evaluate(input_data_t) const;
    virtual double evaluate(input_data_t, output_data_t) const;
    virtual std::string identifier() const;
private:
    reward_t reward;
    int delay;
};

class AndFeature: public Feature {
public:
    AndFeature(const Feature& f1 = NullFeature(), const Feature& f2 = NullFeature(), const Feature& f3 = NullFeature(), const Feature& f4 = NullFeature(), const Feature& f5 = NullFeature());
    virtual ~AndFeature();
    virtual double evaluate(input_data_t) const;
    virtual double evaluate(input_data_t, output_data_t) const;
    virtual std::string identifier() const;
};

#endif /* FEATURE_H_ */
