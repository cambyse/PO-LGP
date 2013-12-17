#ifndef MINIMALREWARD_H_
#define MINIMALREWARD_H_

#include "../../AbstractReward.h"

#include "../../debug.h"

class MinimalReward: public AbstractReward {
public:
    enum REWARD { NO_REWARD, SOME_REWARD } reward;
    MinimalReward(REWARD o = NO_REWARD) {
        reward = o;
        set_type(REWARD_TYPE::MINIMAL);
    }
    virtual ~MinimalReward() = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(MinimalReward);
    virtual ptr_t next() const override {
        if(reward==NO_REWARD) {
            return ptr_t(new MinimalReward(SOME_REWARD));
        } else {
            return ptr_t(new AbstractReward());
        }
    }
    virtual bool operator!=(const AbstractReward &other) const override {
        if(this->get_type()!=other.get_type()) {
            return true;
        } else {
            auto minimal_reward = dynamic_cast<const MinimalReward *>(&other);
            if(minimal_reward==nullptr) {
                DEBUG_ERROR("Dynamic cast failed");
                return true;
            } else {
                return this->reward!=minimal_reward->reward;
            }
        }
    }
    virtual bool operator<(const AbstractReward &other) const override {
        if(this->get_type()<other.get_type()) {
            return true;
        } else {
            auto minimal_reward = dynamic_cast<const MinimalReward *>(&other);
            if(minimal_reward==nullptr) {
                DEBUG_ERROR("Dynamic cast failed");
                return true;
            } else {
                return this->reward<minimal_reward->reward;
            }
        }
    }
    virtual const std::string print() const override {
        if(reward==NO_REWARD) return std::string("MinimalReward(NO_REWARD)");
        if(reward==SOME_REWARD) return std::string("MinimalReward(SOME_REWARD)");
        return std::string("MinimalReward(NONE)");
    }
};

#include "../../debug_exclude.h"

#endif /* MINIMALREWARD_H_ */
