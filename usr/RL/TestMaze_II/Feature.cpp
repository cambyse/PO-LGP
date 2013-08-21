#include "Feature.h"

#define DEBUG_LEVEL 1
#include "debug.h"

using std::get;
using std::string;

using util::INVALID;

typedef std::unique_ptr<Feature> unique_f_ptr;

int Feature::field_width[2] = {0,0};
long Feature::id_counter = 0;
Feature::basis_feature_container_t Feature::basis_features = Feature::basis_feature_container_t();

Feature::Feature(): type(ABSTRACT), id(id_counter), complexity(0), subfeatures(0), const_feature(false) {
    ++id_counter;
}

Feature::~Feature() {}

Feature::feature_return_value Feature::evaluate(const_instanceIt_t) const {
    DEBUG_OUT(0,"Error: Evaluating abstract type Feature");
    return 0;
}

Feature::feature_return_value Feature::evaluate(const_instanceIt_t insIt, action_t action, state_t state, reward_t reward) const {
    if(insIt!=INVALID) {
        const instance_t * new_ins = instance_t::create(action,state,reward,insIt);
        Feature::feature_return_value ret = this->evaluate(new_ins);
        delete new_ins;
        return ret;
    } else {
        return 0;
    }
}

Feature::feature_return_value Feature::evaluate(const look_up_map_t&) const {
    DEBUG_OUT(0,"Error: Evaluating abstract type Feature");
    return 0;
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
    return subfeatures.begin();
}

Feature::subfeature_const_iterator_t Feature::get_subfeatures_end() const {
    return subfeatures.end();
}

uint Feature::get_subfeatures_size() const {
    return subfeatures.size();
}

void Feature::clean_up_subfeatures() {
    // make sure subfeatures are sorted
    subfeatures.sort(&(pComp));

    // erase duplicates
    subfeature_const_iterator_t it_1;
    subfeature_iterator_t it_2;
    it_1 = it_2 = subfeatures.begin();
    ++it_2;
    while(it_2!=subfeatures.end()) {
        if(
            **it_2==**it_1 ||
            ( (*it_2)->const_feature && (*it_2)->const_return_value==1 )
            ) {
            it_2 = subfeatures.erase(it_2);
        } else {
            ++it_1;
            ++it_2;
        }
    }
}

ConstFeature::ConstFeature(const long long int& v) {
    type = CONST_FEATURE;
    complexity = 0;
    const_feature = true;
    const_return_value = v;
    subfeatures.push_back(this);
}

ConstFeature * ConstFeature::create(const long long int& v) {
    ConstFeature * new_feature = new ConstFeature(v);
    basis_features.insert(unique_f_ptr(new_feature));
    return new_feature;
}

ConstFeature::~ConstFeature() {}

Feature::feature_return_value ConstFeature::evaluate(const_instanceIt_t insIt) const {
    return insIt==INVALID ? 0 : const_return_value;
}

Feature::feature_return_value ConstFeature::evaluate(const_instanceIt_t insIt, action_t, state_t, reward_t) const {
    // re-implement because it's more efficient
    return insIt==INVALID ? 0 : const_return_value;
}

string ConstFeature::identifier() const {
    QString id_string(" c("
                      +QString(field_width[0]+2,' ')
                      +QString("%1").arg(const_return_value,field_width[1])
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

Feature::feature_return_value ActionFeature::evaluate(const_instanceIt_t insIt) const {
    if( insIt!=INVALID && (insIt+=delay)!=INVALID && insIt->action==action ) {
        return 1;
    } else {
        return 0;
    }
}

string ActionFeature::identifier() const {
    QString id_string(" a("
                      +QString(5-field_width[0],' ')
                      +QString(action_t::action_string(action))
                      +", "
                      +QString("%1").arg(delay,field_width[1])
                      +")"
        );
    return id_string.toStdString()+Feature::identifier();
}

bool ActionFeature::features_contradict(const ActionFeature& f1, const ActionFeature& f2) {
    if(f1.delay==f2.delay && f1.action!=f2.action) {
        return true;
    } else {
        return false;
    }
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

Feature::feature_return_value StateFeature::evaluate(const_instanceIt_t insIt) const {
    if( insIt!=INVALID && (insIt+=delay)!=INVALID && insIt->state==state ) {
        return 1;
    } else {
        return 0;
    }
}

string StateFeature::identifier() const {
    QString id_string(" s("
            +QString("%1").arg(state,field_width[0])
            +", "
            +QString("%1").arg(delay,field_width[1])
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

bool StateFeature::features_contradict(const StateFeature& f1, const StateFeature& f2) {
    if(f1.delay==f2.delay && f1.state!=f2.state) {
        return true;
    } else {
        return false;
    }
}

RelativeStateFeature::RelativeStateFeature(const int& dx, const int& dy, const int& d1, const int& d2):
    delta_x(dx), delta_y(dy), delay_1(d1), delay_2(d2)
{
    type = RELATIVE_STATE;
    complexity = 1;
    subfeatures.push_back(this);
}

RelativeStateFeature::~RelativeStateFeature() {}

RelativeStateFeature * RelativeStateFeature::create(const int& dx, const int& dy, const int& d1, const int& d2) {
    RelativeStateFeature * new_feature = new RelativeStateFeature(dx,dy,d1,d2);
    basis_features.insert(unique_f_ptr(new_feature));
    return new_feature;
}

Feature::feature_return_value RelativeStateFeature::evaluate(const_instanceIt_t insIt) const {
    if(insIt!=INVALID) {
        const_instanceIt_t insIt_1 = insIt+delay_1;
        const_instanceIt_t insIt_2 = insIt+delay_2;
        if(insIt_1!=INVALID && insIt_2!=INVALID) {
            state_t s_1 = insIt_1->state;
            state_t s_2 = insIt_2->state;
            idx_t x_1 = s_1%Config::maze_x_size;
            idx_t y_1 = s_1/Config::maze_x_size;
            idx_t x_2 = s_2%Config::maze_x_size;
            idx_t y_2 = s_2/Config::maze_x_size;
            if(x_1-x_2==delta_x && y_1-y_2==delta_y) {
                return 1;
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}

string RelativeStateFeature::identifier() const {
    QString id_string = QString("rs(dx=%1,dy=%2,i1=%3,i2=%4)")
        .arg(delta_x)
        .arg(delta_y)
        .arg(delay_1)
        .arg(delay_2);
    return id_string.toStdString()+Feature::identifier();
}

bool RelativeStateFeature::features_contradict(const RelativeStateFeature& f1, const RelativeStateFeature& f2) {
    if(
        f1.delay_1!=f2.delay_1 ||
        f1.delay_2!=f2.delay_2 ||
        f1.delta_x!=f2.delta_x ||
        f1.delta_y!=f2.delta_y
        ) {
        return true;
    } else {
        return false;
    }
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

Feature::feature_return_value RewardFeature::evaluate(const_instanceIt_t insIt) const {
    if( insIt!=INVALID && (insIt+=delay)!=INVALID && insIt->reward==reward ) {
        return 1;
    } else {
        return 0;
    }
}

string RewardFeature::identifier() const {
    QString id_string(" r("
            +QString("%1").arg(reward,field_width[0])
            +", "
            +QString("%1").arg(delay,field_width[1])
            +")"
    );
    return id_string.toStdString()+Feature::identifier();
}

bool RewardFeature::features_contradict(const RewardFeature& f1, const RewardFeature& f2) {
    if(f1.delay==f2.delay && f1.reward!=f2.reward) {
        return true;
    } else {
        return false;
    }
}

AndFeature::AndFeature() {
    type = AND;
}

AndFeature::AndFeature(const Feature& f1) {
    type = AND;
    subfeatures.insert(subfeatures.begin(),f1.get_subfeatures_begin(),f1.get_subfeatures_end());
    clean_up_subfeatures();
    complexity = subfeatures.size();
    check_for_contradicting_subfeatures();
}

AndFeature::AndFeature(const Feature& f1, const Feature& f2) {
    type = AND;
    subfeatures.insert(subfeatures.begin(),f1.get_subfeatures_begin(),f1.get_subfeatures_end());
    subfeatures.insert(subfeatures.begin(),f2.get_subfeatures_begin(),f2.get_subfeatures_end());
    clean_up_subfeatures();
    complexity = subfeatures.size();
    check_for_contradicting_subfeatures();
}

AndFeature::~AndFeature() {}

Feature::feature_return_value AndFeature::evaluate(const_instanceIt_t insIt) const {
    if(insIt!=INVALID) {
        if(const_feature) {
            return const_return_value;
        } else {
            Feature::feature_return_value prod = 1;
            for(auto feature_iterator : subfeatures) {
                prod *= feature_iterator->evaluate(insIt);
                if(prod==0) {
                    break;
                }
            }
            return prod;
        }
    } else {
        return 0;
    }
}

AndFeature::feature_return_value AndFeature::evaluate(const look_up_map_t& look_up_map) const {
    Feature::feature_return_value prod = 1;
    for(auto feature_iterator : subfeatures) {
        auto it = look_up_map.find(feature_iterator);
        DEBUG_IF(it==look_up_map.end()) {
            DEBUG_OUT(0,"Error: Subfeature not in look-up map");
            return 0;
        }
        prod *= it->second;
        if(prod==0) {
            break;
        }
    }
    return prod;
}

string AndFeature::identifier() const {
    string id_string(" ^(");
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

void AndFeature::check_for_contradicting_subfeatures() {
    for( auto sub_1 : subfeatures ) {
        for( auto sub_2 : subfeatures ) {
            if(sub_1->get_type()==sub_2->get_type() && sub_1->contradicts(*sub_2)) {
                const_feature = true;
                const_return_value = 0;
                return;
            }
        }
    }
}
