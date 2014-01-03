#include "ListedReward.h"

#include "../util/Macro.h"

#include <sstream>
#include <algorithm> // for min, max

#define DEBUG_LEVEL 2
#include "../debug.h"

using std::min;
using std::max;
using std::string;

ListedReward::ListedReward(const std::vector<value_t> & list, const int & idx):
    reward_list(list), reward_index(idx)
{
    if(DEBUG_LEVEL>0 && reward_index>=(int)reward_list.size()) {
        DEBUG_ERROR("Reward index (" << reward_index << ")out of bounds (0-" << reward_list.size()-1 << ")");
    }
    set_type(REWARD_TYPE::LISTED_REWARD);
}

ListedReward::ListedReward(const std::vector<value_t> & list, const double & value):
    ListedReward(list,0)
{
    set_value(value);
}

ListedReward::Iterator ListedReward::begin() const {
    return Iterator(ptr_t(new ListedReward(reward_list,0)));
}

ListedReward::ptr_t ListedReward::next() const {
    if(reward_index<(int)reward_list.size()-1) {
        return ptr_t(new ListedReward(reward_list,reward_index+1));
    } else {
        return ptr_t(new AbstractReward());
    }
}

bool ListedReward::operator!=(const AbstractReward &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const ListedReward *);
    return (
        this->reward_index != ptr->reward_index ||
        this->reward_list  != ptr->reward_list
        );
}

bool ListedReward::operator<(const AbstractReward &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const ListedReward *);
    return ( this->reward_index < ptr->reward_index ||
             ( this->reward_index == ptr->reward_index &&
               this->reward_list < ptr->reward_list )
        );
}

const string ListedReward::print() const {
    std::stringstream ret;
    if(DEBUG_LEVEL>1) {
        ret << "ListedReward({";
        bool first = true;
        for(value_t v : reward_list) {
            if(first) {
                first = false;
            } else {
                ret << ",";
            }
            ret << v;
        }
        ret << "}," << reward_list[reward_index] << ")";
    } else {
        ret << "ListedReward(" << reward_list[reward_index] << ")";
    }
    return ret.str();
}

void ListedReward::set_value(const value_t& v) {
    int idx = 0;
    for(value_t list_value : reward_list) {
        if(list_value==v) {
            reward_index = idx;
            return;
        }
        ++idx;
    }
    DEBUG_ERROR("Value (" << v << ") does not match any in reward list.");
}

ListedReward::value_t ListedReward::min_reward() const {
    value_t min_val = DBL_MAX;
    for(value_t v : reward_list) {
        min_val = min(min_val,v);
    }
    return min_val;
}

ListedReward::value_t ListedReward::max_reward() const {
    value_t max_val = -DBL_MAX;
    for(value_t v : reward_list) {
        max_val = max(max_val,v);
    }
    return max_val;
}

void ListedReward::set_type(REWARD_TYPE t) {
    AbstractReward::set_type(t);
}
