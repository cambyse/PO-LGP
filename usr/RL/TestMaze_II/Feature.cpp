/*
 * Feature.cpp
 *
 *  Created on: Nov 9, 2012
 *      Author: robert
 */

#include "Feature.h"

#define DEBUG_LEVEL 1
#include "debug.h"

using std::get;
using std::string;

typedef std::unique_ptr<Feature> unique_f_ptr;

int Feature::field_width[2] = {0,0};
long Feature::id_counter = 0;
Feature::basis_feature_container_t Feature::basis_features = Feature::basis_feature_container_t();

Feature::Feature(): type(ABSTRACT), id(id_counter), complexity(0), subfeatures(0) {
    ++id_counter;
}

Feature::~Feature() {}

//string Feature::identifier() const {
//    return QString("(%1)").arg(id).toStdString();
//}

string Feature::identifier() const {
    return string("");
}

Feature::TYPE Feature::get_type() const {
    return type;
}

int Feature::get_id() const{
    return id;
}

unsigned int Feature::get_complexity() const{
    return complexity;
}

bool Feature::operator==(const Feature& other) const {
    if(subfeatures.size()!=other.subfeatures.size()) {
        return false;
    } else {
        subfeature_container_t::const_iterator f1_it=subfeatures.begin();
        subfeature_container_t::const_iterator f2_it=other.subfeatures.begin();
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

bool Feature::operator!=(const Feature& other) const {
    return !(*this==other);
}

bool Feature::operator<(const Feature& other) const {
    if(subfeatures.size()==other.get_subfeatures_size()) {
        subfeature_const_iterator_t this_it  = subfeatures.begin();
        subfeature_const_iterator_t other_it = other.get_subfeatures_begin();
        while(this_it!=subfeatures.end() && other_it!=other.get_subfeatures_end()) {
            if((*this_it)->id==(*other_it)->get_id()) {
                ++this_it;
                ++other_it;
            }
            else {
                return (*this_it)->id < (*other_it)->get_id();
            }
        }
        if(this_it!=subfeatures.end() || other_it!=other.get_subfeatures_end()) {
            DEBUG_OUT(0, "Autsch! This should never happen!");
        }
    } else {
        return subfeatures.size()<other.subfeatures.size();
    }
    return false;
}

bool Feature::pComp(Feature const * first, Feature const * second) {
    return (*first)<(*second);
}

Feature::subfeature_const_iterator_t Feature::get_subfeatures_begin() const {
    if(subfeatures.size()==0) {
        return subfeatures.end();
    } else {
        return subfeatures.begin();
    }
}

Feature::subfeature_const_iterator_t Feature::get_subfeatures_end() const {
    return subfeatures.end();
}

uint Feature::get_subfeatures_size() const {
    return subfeatures.size();
}

void Feature::clean_up_subfeatures() {
    subfeature_const_iterator_t it_1;
    subfeature_iterator_t it_2;
    it_1 = it_2 = subfeatures.begin();
    ++it_2;
    while(it_2!=subfeatures.end()) {
        if(**it_2==**it_1 || (*it_2)->get_type()==NULL_FEATURE) {
            it_2 = subfeatures.erase(it_2);
        } else {
            ++it_1;
            ++it_2;
        }
    }
}

NullFeature::NullFeature(){
    type = NULL_FEATURE;
    complexity = 0;
}

NullFeature::~NullFeature() {}

double NullFeature::evaluate(input_data_t) const {
    return 0;
}

double NullFeature::evaluate(input_data_t, output_data_t) const {
    return 0;
}

string NullFeature::identifier() const {
    QString id_string("n("
            +QString(field_width[0]+field_width[1]+1,' ')
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

ActionFeature::ActionFeature(const action_t& a, const int& d): action(a), delay(d) {
    type = ACTION;
    complexity = 1;
    if( field_width[0] < 5 ) {
        field_width[0]=5;
    }
    if( field_width[1] < log10(abs(delay))+2 ) {
        field_width[1]=log10(abs(delay))+2;
    }
    subfeatures.push_back(this);
}

ActionFeature::~ActionFeature() {}

ActionFeature * ActionFeature::create(const action_t& a, const int& d) {
    ActionFeature * new_feature = new ActionFeature(a,d);
    basis_features.insert(unique_f_ptr(new_feature));
    return new_feature;
}

double ActionFeature::evaluate(input_data_t input_data) const {
    if( (input_data+=delay)->action==action ) {
        return 1;
    } else {
        return 0;
    }
}

double ActionFeature::evaluate(input_data_t input_data, output_data_t) const {
    return evaluate(input_data);
}

string ActionFeature::identifier() const {
    QString id_string("a("
            +QString(5-field_width[0],' ')
            +QString(Data::action_strings[action])
            +", "
            +QString("%1").arg(delay,field_width[1])
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

StateFeature::StateFeature(const state_t& s, const int& d): state(s), delay(d) {
    type = STATE;
    complexity = 1;
    if( field_width[0] < log10(abs(state)) ) {
        field_width[0]=log10(abs(state));
    }
    if( field_width[1] < log10(abs(delay))+2 ) {
        field_width[1]=log10(abs(delay))+2;
    }
    subfeatures.push_back(this);
}

StateFeature::~StateFeature() {}

StateFeature * StateFeature::create(const state_t& s, const int& d) {
    StateFeature * new_feature = new StateFeature(s,d);
    basis_features.insert(unique_f_ptr(new_feature));
    return new_feature;
}

double StateFeature::evaluate(input_data_t input_data) const {
    if( (input_data+=delay)->state==state ) {
        return 1;
    } else {
        return 0;
    }
}

double StateFeature::evaluate(input_data_t input_data, output_data_t output_data) const {
    if( delay==0 ) {
        if( output_data.state==state ) {
            return 1;
        } else {
            return 0;
        }
    } else {
        if( (input_data+=delay)->state==state ) {
            return 1;
        } else {
            return 0;
        }
    }
}

string StateFeature::identifier() const {
    QString id_string("s("
            +QString("%1").arg(state,field_width[0])
            +", "
            +QString("%1").arg(delay,field_width[1])
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

RewardFeature::RewardFeature(const reward_t& r, const int& d): reward(r), delay(d) {
    type = REWARD;
    complexity = 1;
    if( field_width[0] < 2 ) {
        field_width[0]=2;
    }
    if( field_width[1] < log10(abs(delay))+2 ) {
        field_width[1]=log10(abs(delay))+2;
    }
    subfeatures.push_back(this);
}

RewardFeature::~RewardFeature() {}

RewardFeature * RewardFeature::create(const reward_t& r, const int& d) {
    RewardFeature * new_feature = new RewardFeature(r,d);
    basis_features.insert(unique_f_ptr(new_feature));
    return new_feature;
}

double RewardFeature::evaluate(input_data_t input_data) const {
    if( (input_data+=delay)->reward==reward ) {
        return 1;
    } else {
        return 0;
    }
}

double RewardFeature::evaluate(input_data_t input_data, output_data_t output_data) const {
    if( delay==0 ) {
        if( output_data.reward==reward ) {
            return 1;
        } else {
            return 0;
        }
    } else {
        if( (input_data+=delay)->reward==reward ) {
            return 1;
        } else {
            return 0;
        }
    }
}

string RewardFeature::identifier() const {
    QString id_string("r("
            +QString("%1").arg(reward,field_width[0])
            +", "
            +QString("%1").arg(delay,field_width[1])
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

AndFeature::AndFeature(const Feature& f1, const Feature& f2, const Feature& f3, const Feature& f4, const Feature& f5) {
    type = AND;
    subfeatures.insert(subfeatures.begin(),f1.get_subfeatures_begin(),f1.get_subfeatures_end());
    subfeatures.insert(subfeatures.begin(),f2.get_subfeatures_begin(),f2.get_subfeatures_end());
    subfeatures.insert(subfeatures.begin(),f3.get_subfeatures_begin(),f3.get_subfeatures_end());
    subfeatures.insert(subfeatures.begin(),f4.get_subfeatures_begin(),f4.get_subfeatures_end());
    subfeatures.insert(subfeatures.begin(),f5.get_subfeatures_begin(),f5.get_subfeatures_end());
    subfeatures.sort(&(pComp));
    clean_up_subfeatures();
    complexity = subfeatures.size();
}

AndFeature::~AndFeature() {}

double AndFeature::evaluate(input_data_t input_data) const {
    double prod = 1;
    for(subfeature_const_iterator_t feature_iterator=subfeatures.begin();
            feature_iterator!=subfeatures.end();
            ++feature_iterator) {
        prod *= (*feature_iterator)->evaluate(input_data);
    }
    return prod;
}

double AndFeature::evaluate(input_data_t input_data, output_data_t output_data) const {
    double prod = 1;
    for(subfeature_const_iterator_t feature_iterator=subfeatures.begin();
            feature_iterator!=subfeatures.end();
            ++feature_iterator) {
        prod *= (*feature_iterator)->evaluate(input_data, output_data);
    }
    return prod;
}

string AndFeature::identifier() const {
    string id_string("^(");
    bool first = true;
    for(subfeature_const_iterator_t feature_iterator=subfeatures.begin();
            feature_iterator!=subfeatures.end();
            ++feature_iterator) {
        if(!first) {
            id_string += " + ";
        }
        first = false;
        id_string += (*feature_iterator)->identifier();
    }
    return id_string+")"+Feature::identifier();
};
