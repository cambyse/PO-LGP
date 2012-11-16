/*
 * Feature.cpp
 *
 *  Created on: Nov 9, 2012
 *      Author: robert
 */

#include "Feature.h"

#define DEBUG_LEVEL 1
#include "debug.h"

int Feature::field_width[2] = {0,0};
long Feature::id_counter = 0;

Feature::Feature(): id(id_counter) {
    ++id_counter;
}

Feature::~Feature() {}

//std::string Feature::identifier() const {
//    return QString("(%1)").arg(id).toStdString();
//}

std::string Feature::identifier() const {
    return std::string("");
}

int Feature::get_id() {
    return id;
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

bool Feature::operator<(const Feature& other) {
    return id<other.id;
}

bool Feature::pComp(Feature const * first, Feature const * second) {
    return first->id<second->id;
}

NullFeature::NullFeature() {}

NullFeature::~NullFeature() {}

double NullFeature::evaluate(input_data_t) const {
    return 0;
}

double NullFeature::evaluate(input_data_t, output_data_t) const {
    return 0;
}

std::string NullFeature::identifier() const {
    QString id_string("n("
            +QString(field_width[0]+field_width[1]+1,' ')
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

ActionFeature::ActionFeature(const action_t& a, const int& d): action(a), delay(d) {
    if( field_width[0] < 5 ) {
        field_width[0]=5;
    }
    if( field_width[1] < log10(abs(delay))+2 ) {
        field_width[1]=log10(abs(delay))+2;
    }
    subfeatures.push_back(this);
}

ActionFeature::~ActionFeature() {}

double ActionFeature::evaluate(input_data_t data) const {
    if( std::get<0>(*(data+=delay))==action ) {
        return 1;
    } else {
        return 0;
    }
}

double ActionFeature::evaluate(input_data_t data, output_data_t) const {
    return evaluate(data);
}

std::string ActionFeature::identifier() const {
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
    if( field_width[0] < log10(abs(state)) ) {
        field_width[0]=log10(abs(state));
    }
    if( field_width[1] < log10(abs(delay))+2 ) {
        field_width[1]=log10(abs(delay))+2;
    }
    subfeatures.push_back(this);
}

StateFeature::~StateFeature() {}

double StateFeature::evaluate(input_data_t data) const {
    if( std::get<1>(*(data+=delay))==state ) {
        return 1;
    } else {
        return 0;
    }
}

double StateFeature::evaluate(input_data_t data, output_data_t data_predict) const {
    if( delay==0 ) {
        if( std::get<1>(data_predict)==state ) {
            return 1;
        } else {
            return 0;
        }
    } else {
        if( std::get<1>(*(data+=delay))==state ) {
            return 1;
        } else {
            return 0;
        }
    }
}

std::string StateFeature::identifier() const {
    QString id_string("s("
            +QString("%1").arg(state,field_width[0])
            +", "
            +QString("%1").arg(delay,field_width[1])
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

RewardFeature::RewardFeature(const reward_t& r, const int& d): reward(r), delay(d) {
    if( field_width[0] < 2 ) {
        field_width[0]=2;
    }
    if( field_width[1] < log10(abs(delay))+2 ) {
        field_width[1]=log10(abs(delay))+2;
    }
    subfeatures.push_back(this);
}

RewardFeature::~RewardFeature() {}

double RewardFeature::evaluate(input_data_t data) const {
    if( std::get<2>(*(data+=delay))==reward ) {
        return 1;
    } else {
        return 0;
    }
}

double RewardFeature::evaluate(input_data_t data, output_data_t data_predict) const {
    if( delay==0 ) {
        if( std::get<2>(data_predict)==reward ) {
            return 1;
        } else {
            return 0;
        }
    } else {
        if( std::get<2>(*(data+=delay))==reward ) {
            return 1;
        } else {
            return 0;
        }
    }
}

std::string RewardFeature::identifier() const {
    QString id_string("r("
            +QString("%1").arg(reward,field_width[0])
            +", "
            +QString("%1").arg(delay,field_width[1])
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

AndFeature::AndFeature(const Feature& f1, const Feature& f2, const Feature& f3, const Feature& f4, const Feature& f5) {
    subfeatures.insert(subfeatures.begin(),f1.subfeatures.begin(),f1.subfeatures.end());
    subfeatures.insert(subfeatures.begin(),f2.subfeatures.begin(),f2.subfeatures.end());
    subfeatures.insert(subfeatures.begin(),f3.subfeatures.begin(),f3.subfeatures.end());
    subfeatures.insert(subfeatures.begin(),f4.subfeatures.begin(),f4.subfeatures.end());
    subfeatures.insert(subfeatures.begin(),f5.subfeatures.begin(),f5.subfeatures.end());
    subfeatures.sort(&(pComp));
}

AndFeature::~AndFeature() {}

double AndFeature::evaluate(input_data_t data) const {
    double prod = 1;
    for(subfeature_iterator feature_iterator=subfeatures.begin();
            feature_iterator!=subfeatures.end();
            ++feature_iterator) {
        prod *= (*feature_iterator)->evaluate(data);
    }
    return prod;
}

double AndFeature::evaluate(input_data_t data, output_data_t data_predict) const {
    double prod = 1;
    for(subfeature_iterator feature_iterator=subfeatures.begin();
            feature_iterator!=subfeatures.end();
            ++feature_iterator) {
        prod *= (*feature_iterator)->evaluate(data, data_predict);
    }
    return prod;
}

std::string AndFeature::identifier() const {
    std::string id_string;
    bool first = true;
    for(subfeature_iterator feature_iterator=subfeatures.begin();
            feature_iterator!=subfeatures.end();
            ++feature_iterator) {
        if(!first) {
            id_string += " + ";
        }
        first = false;
        id_string += (*feature_iterator)->identifier();
    }
    return id_string+Feature::identifier();
};
