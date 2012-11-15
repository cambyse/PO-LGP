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

#include "debug.h"

class Feature {
protected:
    typedef std::list<Feature * > subfeature_container_t;
    typedef Data::input_data_t    input_data_t;
    typedef Data::output_data_t   output_data_t;
    typedef Data::action_t        action_t;
    typedef Data::state_t         state_t;
    typedef Data::reward_t        reward_t;
public:
    subfeature_container_t subfeatures;
    Feature(): id(id_counter) { ++id_counter; }
    virtual ~Feature() {}
    virtual double evaluate(input_data_t) const = 0;
    virtual double evaluate(input_data_t, output_data_t) const = 0;
//    virtual std::string identifier() const { return QString("(%1)").arg(id).toStdString(); }
    virtual std::string identifier() const { return std::string(""); }
    int get_id() { return id; }
    bool operator==(const Feature& other) const {
        if(subfeatures.size()!=other.subfeatures.size()) {
            return false;
        } else {
            typename subfeature_container_t::const_iterator f1_it=subfeatures.begin();
            typename subfeature_container_t::const_iterator f2_it=other.subfeatures.begin();
            while(f1_it!=subfeatures.end() && f2_it!=other.subfeatures.end()) {
                if((*f1_it)->id!=(*f2_it)->id) {
                    return false;
                }
                ++f1_it;
                ++f2_it;
            }
        }
        return true;
    }
    bool operator<(const Feature& other) { return id<other.id; }
    static bool pComp(Feature const * first, Feature const * second) { return first->id<second->id; }
protected:
    long id;
    static int field_width[2];
    static long id_counter;
};

class NullFeature: public Feature {
public:
    NullFeature() {}
    virtual ~NullFeature() {}
    virtual double evaluate(input_data_t) const { return 0; }
    virtual double evaluate(input_data_t, output_data_t) const { return 0; }
    virtual std::string identifier() const { return ("n("+QString(this->field_width[0]+this->field_width[1]+1,' ')+")").toStdString()+Feature::identifier(); }
};

class ActionFeature: public Feature {
public:
    ActionFeature(const action_t& a, const int& d):
        action(a), delay(d) {
        if(this->field_width[0]<5) this->field_width[0]=5;
        if(this->field_width[1]<log10(abs(delay))+2) this->field_width[1]=log10(abs(delay))+2;
        this->subfeatures.push_back(this);
    }
    virtual ~ActionFeature() {}
    virtual double evaluate(input_data_t data) const { return std::get<0>(*(data+=delay))==action ? 1 : 0; }
    virtual double evaluate(input_data_t data, output_data_t) const {
        return evaluate(data);
    }
    virtual std::string identifier() const { return ("a("+QString(5-this->field_width[0],' ')+QString(Data::action_strings[action])+", "+QString("%1").arg(delay,this->field_width[1])+")").toStdString()+Feature::identifier(); }
private:
    action_t action;
    int delay;
};

class StateFeature: public Feature {
public:
    StateFeature(const state_t& s, const int& d):
        state(s), delay(d) {
        if(this->field_width[0]<log10(abs(state))) this->field_width[0]=log10(abs(state));
        if(this->field_width[1]<log10(abs(delay))+2) this->field_width[1]=log10(abs(delay))+2;
        this->subfeatures.push_back(this);
    }
    virtual ~StateFeature() {}
    virtual double evaluate(input_data_t data) const { return std::get<1>(*(data+=delay))==state ? 1 : 0; }
    virtual double evaluate(input_data_t data, output_data_t data_predict) const { return delay==0 ? ( std::get<1>(data_predict)==state ? 1 : 0 ) : ( std::get<1>(*(data+=delay))==state ? 1 : 0 ) ; }
    virtual std::string identifier() const { return ("s("+QString("%1").arg(state,this->field_width[0])+", "+QString("%1").arg(delay,this->field_width[1])+")").toStdString()+Feature::identifier(); }
private:
    state_t state;
    int delay;
};

class RewardFeature: public Feature {
public:
    RewardFeature(const reward_t& r, const int& d):
        reward(r), delay(d) {
        if(this->field_width[0]<2) this->field_width[0]=2;
        if(this->field_width[1]<log10(abs(delay))+2) this->field_width[1]=log10(abs(delay))+2;
        this->subfeatures.push_back(this);
    }
    virtual ~RewardFeature() {}
    virtual double evaluate(input_data_t data) const { return std::get<2>(*(data+=delay))==reward ? 1 : 0; }
    virtual double evaluate(input_data_t data, output_data_t data_predict) const { return delay==0 ? ( std::get<2>(data_predict)==reward ? 1 : 0 ) : ( std::get<2>(*(data+=delay))==reward ? 1 : 0 ) ; }
    virtual std::string identifier() const { return ("r("+QString("%1").arg(reward,this->field_width[0])+", "+QString("%1").arg(delay,this->field_width[1])+")").toStdString()+Feature::identifier(); }
private:
    reward_t reward;
    int delay;
};

class AndFeature: public Feature {
public:
    AndFeature(const Feature& f1, const Feature& f2, const Feature& f3 = NullFeature(), const Feature& f4 = NullFeature(), const Feature& f5 = NullFeature()) {
        this->subfeatures.insert(this->subfeatures.begin(),f1.subfeatures.begin(),f1.subfeatures.end());
        this->subfeatures.insert(this->subfeatures.begin(),f2.subfeatures.begin(),f2.subfeatures.end());
        this->subfeatures.insert(this->subfeatures.begin(),f3.subfeatures.begin(),f3.subfeatures.end());
        this->subfeatures.insert(this->subfeatures.begin(),f4.subfeatures.begin(),f4.subfeatures.end());
        this->subfeatures.insert(this->subfeatures.begin(),f5.subfeatures.begin(),f5.subfeatures.end());
        this->subfeatures.sort(&(this->pComp));
    }
    virtual ~AndFeature() {}
    virtual double evaluate(input_data_t data) const {
        double prod = 1;
        for(typename Feature::subfeature_container_t::const_iterator feature_iterator=this->subfeatures.begin();
                feature_iterator!=this->subfeatures.end();
                ++feature_iterator) {
            prod *= (*feature_iterator)->evaluate(data);
        }
        return prod;
    }
    virtual double evaluate(input_data_t data, output_data_t data_predict) const {
        double prod = 1;
        for(typename Feature::subfeature_container_t::const_iterator feature_iterator=this->subfeatures.begin();
                feature_iterator!=this->subfeatures.end();
                ++feature_iterator) {
            prod *= (*feature_iterator)->evaluate(data, data_predict);
        }
        return prod;
    }
    virtual std::string identifier() const {
        std::string id_string;
        bool first = true;
        for(typename Feature::subfeature_container_t::const_iterator feature_iterator=this->subfeatures.begin();
                feature_iterator!=this->subfeatures.end();
                ++feature_iterator) {
            if(!first) id_string += " + ";
            first = false;
            id_string += (*feature_iterator)->identifier();
        }
        return id_string+Feature::identifier();
    };
};

#include "debug_exclude.h"

#endif /* FEATURE_H_ */
