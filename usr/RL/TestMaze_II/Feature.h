/*
 * Feature.h
 *
 *  Created on: Nov 8, 2012
 *      Author: robert
 */

#ifndef FEATURE_H_
#define FEATURE_H_

#include <QtCore/QString>

#include <list>
#include <string>

#include "Data.h"

class Feature {

protected:
    typedef std::list<Feature * >                  subfeature_container_t;
    typedef subfeature_container_t::const_iterator subfeature_iterator;
    typedef Data::input_data_t                     input_data_t;
    typedef Data::output_data_t                    output_data_t;
    typedef Data::action_t                         action_t;
    typedef Data::state_t                          state_t;
    typedef Data::reward_t                         reward_t;

public:
    subfeature_container_t subfeatures;

    Feature();
    virtual ~Feature();

    virtual double evaluate(input_data_t) const = 0;
    virtual double evaluate(input_data_t, output_data_t) const = 0;
    virtual std::string identifier() const;

    int get_id() const;
    unsigned int get_complexity() const;
    bool operator==(const Feature& other) const;
    bool operator<(const Feature& other);
    static bool pComp(Feature const * first, Feature const * second);

protected:
    long id;
    unsigned int complexity;
    static int field_width[2];
    static long id_counter;
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

public:
    ActionFeature(const action_t& a, const int& d);
    virtual ~ActionFeature();
    virtual double evaluate(input_data_t data) const;
    virtual double evaluate(input_data_t data, output_data_t) const;
    virtual std::string identifier() const;
private:
    action_t action;
    int delay;
};

class StateFeature: public Feature {
public:
    StateFeature(const state_t& s, const int& d);
    virtual ~StateFeature();
    virtual double evaluate(input_data_t data) const;
    virtual double evaluate(input_data_t data, output_data_t data_predict) const;
    virtual std::string identifier() const;
private:
    state_t state;
    int delay;
};

class RewardFeature: public Feature {
public:
    RewardFeature(const reward_t& r, const int& d);
    virtual ~RewardFeature();
    virtual double evaluate(input_data_t data) const;
    virtual double evaluate(input_data_t data, output_data_t data_predict) const;
    virtual std::string identifier() const;
private:
    reward_t reward;
    int delay;
};

class AndFeature: public Feature {
public:
    AndFeature(const Feature& f1, const Feature& f2 = NullFeature(), const Feature& f3 = NullFeature(), const Feature& f4 = NullFeature(), const Feature& f5 = NullFeature());
    virtual ~AndFeature();
    virtual double evaluate(input_data_t data) const;
    virtual double evaluate(input_data_t data, output_data_t data_predict) const;
    virtual std::string identifier() const;
};

#endif /* FEATURE_H_ */
