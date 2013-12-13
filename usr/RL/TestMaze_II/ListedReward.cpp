#include "ListedReward.h"

#include <sstream>

#include "../debug.h"

ListedReward::ListedReward(const std::vector<value_t> & list, const uint & idx):
    reward_list(list), reward_index(idx)
{
    set_type(REWARD_TYPE::LISTED_REWARD);
}

ListedReward::Iterator ListedReward::begin() const {
    return Iterator(ptr_t(new ListedReward(reward_list,0)));
}

ListedReward::ptr_t ListedReward::next() const {
    if(reward_index<reward_list.size()-1) {
        return ptr_t(new ListedReward(reward_list,reward_index+1));
    } else {
        return ptr_t(new ListedReward());
    }
}

bool ListedReward::operator!=(const AbstractReward &other) const {
    if(this->get_type()!=other.get_type()) {
        return true;
    } else {
        auto listed_reward = dynamic_cast<const ListedReward *>(&other);
        if(listed_reward==nullptr) {
            DEBUG_ERROR("Dynamic cast failed");
            return true;
        } else {
            return (
                this->reward_index != listed_reward->reward_index ||
                this->reward_list  != listed_reward->reward_list
                );
        }
    }
}

bool ListedReward::operator<(const AbstractReward &other) const {
    if(this->get_type()<other.get_type()) {
        return true;
    } else {
        auto listed_reward = dynamic_cast<const ListedReward *>(&other);
        if(listed_reward==nullptr) {
            DEBUG_ERROR("Dynamic cast failed");
            return true;
        } else {
            return (
                this->reward_index < listed_reward->reward_index || (
                    this->reward_index == listed_reward->reward_index &&
                    this->reward_list < listed_reward->reward_list
                    )
                );
        }
    }
}

const char * ListedReward::print() const {
    std::stringstream ret;
    ret << "ListedReward(" << reward_list[reward_index] << ")";
    return ret.str().c_str();
}

void ListedReward::set_value(const value_t& v) {
    uint idx = 0;
    for(value_t list_value : reward_list) {
        if(list_value==v) {
            reward_index = idx;
            return;
        }
        ++idx;
    }
    DEBUG_ERROR("Value (" << v << ")does not match any in reward list.");
}

void ListedReward::set_type(REWARD_TYPE t) {
    AbstractReward::set_type(t);
}
