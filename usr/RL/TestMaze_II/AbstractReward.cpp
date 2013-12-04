#include "AbstractReward.h"

#include "debug.h"

AbstractReward::AbstractReward(REWARD_TYPE t):
    reward_type(t)
{}

AbstractReward::Iterator AbstractReward::begin() const {
    return Iterator(ptr_t(new AbstractReward()));
}

AbstractReward::Iterator AbstractReward::end() const {
    return Iterator(ptr_t(new AbstractReward()));
}

AbstractReward::ptr_t AbstractReward::next() const {
    return ptr_t(new AbstractReward());
}

bool AbstractReward::operator!=(const AbstractIteratableSpace& other) const {
    auto abstract_reward = dynamic_cast<const AbstractReward *>(&other);
    if(abstract_reward==nullptr) {
        DEBUG_ERROR("Dynamic cast failed");
        return true;
    } else {
        return *this!=*abstract_reward;
    }
}

bool AbstractReward::operator!=(const AbstractReward& other) const {
    return this->reward_type!=other.reward_type;
}

std::string AbstractReward::print() const {
    return std::string("AbstractReward()");
}

std::ostream& operator<<(std::ostream& out, const AbstractReward& a) {
    return out << a.print();
}

AbstractReward::REWARD_TYPE AbstractReward::get_type() const {
    return reward_type;
}

void AbstractReward::set_type(REWARD_TYPE t) {
    reward_type = t;
}
