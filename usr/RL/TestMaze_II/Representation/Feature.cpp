#include "Feature.h"

#include "DoublyLinkedInstance.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using std::get;
using std::string;
using std::shared_ptr;
using std::dynamic_pointer_cast;

using util::INVALID;

Feature::Feature():
    feature_type(ABSTRACT),
    complexity(0),
    const_feature(false)
{}

Feature::~Feature() {}

Feature::feature_return_t Feature::evaluate(const_instance_ptr_t) const {
    DEBUG_ERROR("Evaluating abstract type Feature");
    return return_function(0);
}

Feature::feature_return_t Feature::evaluate(const_instance_ptr_t ins,
                                                action_ptr_t action,
                                                observation_ptr_t observation,
                                                reward_ptr_t reward) const {
    return this->evaluate(
        DoublyLinkedInstance::create(action,observation,reward,ins,INVALID)
        );
}

Feature::feature_return_t Feature::evaluate(const look_up_map_t&) const {
    DEBUG_ERROR("Evaluating abstract type Feature");
    return return_function(0);
}

// string Feature::identifier() const {
//    return QString("(%1)").arg(id).toStdString();
// }

string Feature::identifier() const {
    return string("");
}

std::ostream& operator<<(std::ostream &out, const Feature& f) {
    return out << f.identifier();
}

bool Feature::operator==(const Feature& other) const {
    return this->feature_type==other.feature_type;
}

bool Feature::operator!=(const Feature& other) const {
    return !(*this==other);
}

bool Feature::operator<(const Feature&) const {
    // sould never be used and will produce an error in unit tests
    return false;
}

Feature::FEATURE_TYPE Feature::get_feature_type() const {
    return feature_type;
}

unsigned int Feature::get_complexity() const{
    return complexity;
}

Feature::feature_return_t Feature::return_function(const feature_return_t& ret) const {
    return ret;
    //return (complexity+1) * ret;
}

ConstFeature::ConstFeature(const feature_return_t& v) {
    feature_type = CONST_FEATURE;
    complexity = 0;
    const_feature = true;
    const_return_value = v;
}

Feature::const_feature_ptr_t ConstFeature::create(const feature_return_t& v) {
    ConstFeature * new_feature = new ConstFeature(v);
    const_feature_ptr_t return_ptr(new_feature);
    new_feature->set_this_ptr(return_ptr);
    return return_ptr;
}

ConstFeature::~ConstFeature() {}

Feature::feature_return_t ConstFeature::evaluate(const_instance_ptr_t ins) const {
    return return_function(ins==INVALID ? 0 : const_return_value);
}

Feature::feature_return_t ConstFeature::evaluate(const_instance_ptr_t ins, action_ptr_t, observation_ptr_t, reward_ptr_t) const {
    // re-implement because it's more efficient
    return return_function(ins==INVALID ? 0 : const_return_value);
}

string ConstFeature::identifier() const {
    return QString("c(%1)").arg(const_return_value).toStdString()+Feature::identifier();
}

bool ConstFeature::operator==(const Feature& other) const {
    const ConstFeature * pt = dynamic_cast<const ConstFeature *>(&other);
    if(pt==nullptr) {
        return false;
    } else {
        return this->const_return_value==pt->const_return_value;
    }
}

bool ConstFeature::operator<(const Feature& other) const {
    if(this->get_feature_type()!=other.get_feature_type()) {
        return this->get_feature_type()<other.get_feature_type();
    } else {
        const ConstFeature * f_ptr = dynamic_cast<const ConstFeature *>(&other);
        if(f_ptr==nullptr) {
            DEBUG_DEAD_LINE;
            return false;
        } else {
            return this->const_return_value<f_ptr->const_return_value;
        }
    }
}

ActionFeature::ActionFeature(const action_ptr_t& a, const int& d): action(a), delay(d) {
    feature_type = ACTION;
    complexity = 1;
}

ActionFeature::~ActionFeature() {}

Feature::const_feature_ptr_t ActionFeature::create(const action_ptr_t& a, const int& d) {
    ActionFeature * new_feature = new ActionFeature(a,d);
    const_feature_ptr_t return_ptr(new_feature);
    new_feature->set_this_ptr(return_ptr);
    return return_ptr;
}

Feature::feature_return_t ActionFeature::evaluate(const_instance_ptr_t ins) const {
    ins = ins->const_next(delay);
    if(ins!=INVALID && ins->action==action) {
        return return_function(1);
    } else {
        return return_function(0);
    }
}

string ActionFeature::identifier() const {
    return QString("a(%1,%2)").arg(action->print().c_str()).arg(delay).toStdString()+Feature::identifier();
}

bool ActionFeature::features_contradict(const ActionFeature& f1, const ActionFeature& f2) {
    if(f1.delay==f2.delay && *(f1.action)!=*(f2.action)) {
        return true;
    } else {
        return false;
    }
}

bool ActionFeature::operator==(const Feature& other) const {
    const ActionFeature * pt = dynamic_cast<const ActionFeature *>(&other);
    if(pt==nullptr) {
        return false;
    } else {
        return (*(this->action)==*(pt->action) && this->delay==pt->delay);
    }
}

bool ActionFeature::operator<(const Feature& other) const {
    if(this->get_feature_type()!=other.get_feature_type()) {
        return this->get_feature_type()<other.get_feature_type();
    } else {
        const ActionFeature * f_ptr = dynamic_cast<const ActionFeature *>(&other);
        if(f_ptr==nullptr) {
            DEBUG_DEAD_LINE;
            return false;
        } else {
            return this->action<f_ptr->action ||
                ( this->action==f_ptr->action && this->delay<f_ptr->delay);
        }
    }
}

ObservationFeature::ObservationFeature(const observation_ptr_t& s, const int& d): observation(s), delay(d) {
    feature_type = OBSERVATION;
    complexity = 1;
}

ObservationFeature::~ObservationFeature() {}

Feature::const_feature_ptr_t ObservationFeature::create(const observation_ptr_t& s, const int& d) {
    ObservationFeature * new_feature = new ObservationFeature(s,d);
    const_feature_ptr_t return_ptr(new_feature);
    new_feature->set_this_ptr(return_ptr);
    return return_ptr;
}

Feature::feature_return_t ObservationFeature::evaluate(const_instance_ptr_t ins) const {
    ins = ins->const_next(delay);
    if(ins!=INVALID && ins->observation==observation ) {
        return return_function(1);
    } else {
        return return_function(0);
    }
}

string ObservationFeature::identifier() const {
    return QString("o(%1,%2)").arg(observation->print().c_str()).arg(delay).toStdString()+Feature::identifier();
}

bool ObservationFeature::features_contradict(const ObservationFeature& f1, const ObservationFeature& f2) {
    if(f1.delay==f2.delay && *(f1.observation)!=*(f2.observation)) {
        return true;
    } else {
        return false;
    }
}

bool ObservationFeature::operator==(const Feature& other) const {
    const ObservationFeature * pt = dynamic_cast<const ObservationFeature *>(&other);
    if(pt==nullptr) {
        return false;
    } else {
        return (*(this->observation)==*(pt->observation) && this->delay==pt->delay);
    }
}

bool ObservationFeature::operator<(const Feature& other) const {
    if(this->get_feature_type()!=other.get_feature_type()) {
        return this->get_feature_type()<other.get_feature_type();
    } else {
        const ObservationFeature * f_ptr = dynamic_cast<const ObservationFeature *>(&other);
        if(f_ptr==nullptr) {
            DEBUG_DEAD_LINE;
            return false;
        } else {
            return this->observation<f_ptr->observation ||
                ( this->observation==f_ptr->observation && this->delay<f_ptr->delay);
        }
    }
}

RewardFeature::RewardFeature(const reward_ptr_t& r, const int& d): reward(r), delay(d) {
    feature_type = REWARD;
    complexity = 1;
}

RewardFeature::~RewardFeature() {}

Feature::const_feature_ptr_t RewardFeature::create(const reward_ptr_t& r, const int& d) {
    RewardFeature * new_feature = new RewardFeature(r,d);
    const_feature_ptr_t return_ptr(new_feature);
    new_feature->set_this_ptr(return_ptr);
    return return_ptr;
}

Feature::feature_return_t RewardFeature::evaluate(const_instance_ptr_t ins) const {
    ins = ins->const_next(delay);
    if(ins!=INVALID && ins->reward==reward ) {
        return return_function(1);
    } else {
        return return_function(0);
    }
}

string RewardFeature::identifier() const {
    return QString("r(%1,%2)").arg(reward->print().c_str()).arg(delay).toStdString()+Feature::identifier();
}

bool RewardFeature::features_contradict(const RewardFeature& f1, const RewardFeature& f2) {
    if(f1.delay==f2.delay && *(f1.reward)!=*(f2.reward)) {
        return true;
    } else {
        return false;
    }
}

bool RewardFeature::operator==(const Feature& other) const {
    const RewardFeature * pt = dynamic_cast<const RewardFeature *>(&other);
    if(pt==nullptr) {
        return false;
    } else {
        return (*(this->reward)==*(pt->reward) && this->delay==pt->delay);
    }
}

bool RewardFeature::operator<(const Feature& other) const {
    if(this->get_feature_type()!=other.get_feature_type()) {
        return this->get_feature_type()<other.get_feature_type();
    } else {
        const RewardFeature * f_ptr = dynamic_cast<const RewardFeature *>(&other);
        if(f_ptr==nullptr) {
            DEBUG_DEAD_LINE;
            return false;
        } else {
            return this->reward<f_ptr->reward ||
                ( this->reward==f_ptr->reward && this->delay<f_ptr->delay);
        }
    }
}

AndFeature::AndFeature() {
    finalize_construction();
}

AndFeature::AndFeature(const_feature_ptr_t f) {
    add_feature(f);
    finalize_construction();
}

AndFeature::AndFeature(const_feature_ptr_t f1, const_feature_ptr_t f2) {
    add_feature(f1);
    add_feature(f2);
    finalize_construction();
}

AndFeature::~AndFeature() {}

Feature::feature_return_t AndFeature::evaluate(const_instance_ptr_t ins) const {
    if(ins!=INVALID) {
        if(const_feature) {
            return return_function(const_return_value);
        } else {
            Feature::feature_return_t prod = 1;
            for(auto feature_iterator : subfeatures) {
                prod *= feature_iterator->evaluate(ins);
                if(prod==0) {
                    break;
                }
            }
            return return_function(prod);
        }
    } else {
        return return_function(0);
    }
}

AndFeature::feature_return_t AndFeature::evaluate(const look_up_map_t& look_up_map) const {
    Feature::feature_return_t prod = 1;
    for(auto feature_iterator : subfeatures) {
        auto it = look_up_map.find(feature_iterator);
        DEBUG_IF(it==look_up_map.end()) {
            DEBUG_ERROR("Subfeature not in look-up map");
            return return_function(0);
        }
        prod *= it->second;
        if(prod==0) {
            break;
        }
    }
    return return_function(prod);
}

string AndFeature::identifier() const {
    string id_string("^(");
    bool first = true;
    for(auto feature_iterator=subfeatures.begin();
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

bool AndFeature::operator==(const Feature& other) const {
    const AndFeature * pt = dynamic_cast<const AndFeature *>(&other);
    if(pt==nullptr) {
        return false;
    } else {
        return (this->subfeatures==pt->subfeatures);
    }
}

bool AndFeature::operator<(const Feature& other) const {
    if(this->get_feature_type()!=other.get_feature_type()) {
        return this->get_feature_type()<other.get_feature_type();
    } else {
        const AndFeature * f_ptr = dynamic_cast<const AndFeature *>(&other);
        if(f_ptr==nullptr) {
            DEBUG_DEAD_LINE;
            return false;
        } else {
            return *this<*f_ptr;
        }
    }
}

bool AndFeature::operator<(const AndFeature& other) const {
    if(subfeatures.size()==other.subfeatures.size()) {
        auto this_it  = subfeatures.begin();
        auto other_it = other.subfeatures.begin();
        while(this_it!=subfeatures.end() && other_it!=other.subfeatures.end()) {
            if(*this_it==*other_it) {
                ++this_it;
                ++other_it;
            }
            else {
                return *this_it<*other_it;
            }
        }
        if(this_it!=subfeatures.end() || other_it!=other.subfeatures.end()) {
            DEBUG_DEAD_LINE;
        }
    } else {
        return subfeatures.size()<other.subfeatures.size();
    }
    // identical subfeatures
    return false;
}

void AndFeature::add_feature(const_feature_ptr_t f) {
    DEBUG_OUT(4,"Feature to add: " << *f);
    typedef shared_ptr<const AndFeature> const_and_f_ptr_t;
    typedef shared_ptr<const BasisFeature> const_basis_f_ptr_t;
    const_and_f_ptr_t and_feature = dynamic_pointer_cast<const AndFeature>(f);
    if(and_feature!=const_and_f_ptr_t()) {
        subfeatures.insert(
            and_feature->subfeatures.begin(),
            and_feature->subfeatures.end()
            );
    } else {
        const_basis_f_ptr_t basis_feature = dynamic_pointer_cast<const BasisFeature>(f);
        if(basis_feature!=const_basis_f_ptr_t()) {
            subfeatures.insert(const_feature_ptr_t(basis_feature->get_this_ptr()));
        } else {
            DEBUG_ERROR("Unsupported feature type");
            DEBUG_DEAD_LINE;
        }
    }
}

void AndFeature::finalize_construction() {
    feature_type = AND;
    complexity = subfeatures.size();
    check_for_contradicting_subfeatures();
}

void AndFeature::check_for_contradicting_subfeatures() {
    for( auto sub_1 : subfeatures ) {
        for( auto sub_2 : subfeatures ) {
            if(sub_1->get_feature_type()==sub_2->get_feature_type() && sub_1->contradicts(*sub_2)) {
                const_feature = true;
                const_return_value = 0;
                return;
            }
        }
    }
}

AndFeature::feature_return_t AndFeature::return_function(const feature_return_t& ret) const {
    return ret;
    //return ret/(complexity+1);
}
