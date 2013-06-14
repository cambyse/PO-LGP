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

#include "Config.h"
#include "Representation/Representation.h"

class Feature {
public:
    USE_CONFIG_TYPEDEFS;
    USE_REPRESENTATION_TYPEDEFS;
    typedef std::list<Feature * >                  subfeature_container_t;
    typedef subfeature_container_t::iterator       subfeature_iterator_t;
    typedef subfeature_container_t::const_iterator subfeature_const_iterator_t;
    typedef std::set<std::unique_ptr<Feature> >    basis_feature_container_t;
    typedef double                                 feature_return_value;
    enum TYPE { ABSTRACT, CONST_FEATURE, NULL_FEATURE, ACTION, STATE, REWARD, AND };
    Feature();
    virtual ~Feature();
    virtual feature_return_value evaluate(const instance_t *) const;
    virtual feature_return_value evaluate(const instance_t *, action_t, state_t, reward_t) const;
    virtual std::string identifier() const;
    friend std::ostream& operator<<(std::ostream&, const Feature&);
    virtual TYPE get_type() const;
    virtual int get_id() const;
    virtual unsigned int get_complexity() const;
    virtual bool operator==(const Feature& other) const;
    virtual bool operator!=(const Feature& other) const;
    virtual bool operator<(const Feature& other) const;
    static bool pComp(Feature const * first, Feature const * second);
    virtual subfeature_const_iterator_t get_subfeatures_begin() const;
    virtual subfeature_const_iterator_t get_subfeatures_end() const;
    virtual uint get_subfeatures_size() const;
    virtual bool is_const_feature() const { return const_feature; }
    virtual bool contradicts(const Feature&) { return false; }
protected:
    TYPE type;
    long id;
    unsigned int complexity;
    static int field_width[2];
    static long id_counter;
    subfeature_container_t subfeatures;
    static basis_feature_container_t basis_features;
    bool const_feature;
    feature_return_value const_return_value;
    virtual void clean_up_subfeatures();
};

class NullFeature: public Feature {
public:
    NullFeature();
    virtual ~NullFeature();
    virtual feature_return_value evaluate(const instance_t *) const;
    virtual feature_return_value evaluate(const instance_t *, action_t, state_t, reward_t) const;
    virtual std::string identifier() const;
};

class ConstFeature: public Feature {
private:
    ConstFeature(const long long int& v);
    virtual ~ConstFeature();
public:
    static ConstFeature * create(const long long int& v = 0);
    virtual feature_return_value evaluate(const instance_t *) const;
    virtual feature_return_value evaluate(const instance_t *, action_t, state_t, reward_t) const;
    virtual std::string identifier() const;
};

class ActionFeature: public Feature {
private:
    ActionFeature(const action_t& a, const int& d);
    virtual ~ActionFeature();
public:
    static ActionFeature * create(const action_t& a, const int& d);
    virtual feature_return_value evaluate(const instance_t *) const;
    virtual feature_return_value evaluate(const instance_t *, action_t, state_t, reward_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const ActionFeature& f1, const ActionFeature& f2);
    bool contradicts(const ActionFeature& f) { return features_contradict(*this,f); }
protected:
    action_t action;
    int delay;
};

class StateFeature: public Feature {
private:
    StateFeature(const state_t& s, const int& d);
    virtual ~StateFeature();
public:
    static StateFeature * create(const state_t& s, const int& d);
    virtual feature_return_value evaluate(const instance_t *) const;
    virtual feature_return_value evaluate(const instance_t *, action_t, state_t, reward_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const StateFeature& f1, const StateFeature& f2);
    bool contradicts(const StateFeature& f) { return features_contradict(*this,f); }
protected:
    state_t state;
    int delay;
};

class RewardFeature: public Feature {
private:
    RewardFeature(const reward_t& r, const int& d);
    virtual ~RewardFeature();
public:
    static RewardFeature * create(const reward_t& r, const int& d);
    virtual feature_return_value evaluate(const instance_t *) const;
    virtual feature_return_value evaluate(const instance_t *, action_t, state_t, reward_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const RewardFeature& f1, const RewardFeature& f2);
    bool contradicts(const RewardFeature& f) { return features_contradict(*this,f); }
protected:
    reward_t reward;
    int delay;
};

class AndFeature: public Feature {
public:
    AndFeature(const Feature& f1 = NullFeature(), const Feature& f2 = NullFeature(), const Feature& f3 = NullFeature(), const Feature& f4 = NullFeature(), const Feature& f5 = NullFeature());
    virtual ~AndFeature();
    virtual feature_return_value evaluate(const instance_t *) const;
    virtual feature_return_value evaluate(const instance_t *, action_t, state_t, reward_t) const;
    virtual std::string identifier() const;
};

#endif /* FEATURE_H_ */
