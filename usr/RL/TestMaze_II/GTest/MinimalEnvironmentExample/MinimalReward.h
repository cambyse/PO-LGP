#ifndef MINIMALREWARD_H_
#define MINIMALREWARD_H_

#include "../../AbstractReward.h"

#include "../../debug.h"

class MinimalReward: public AbstractReward {
public:
    enum REWARD { NO_REWARD, SOME_REWARD, NONE } reward;
    MinimalReward(REWARD o = NONE) {
        reward = o;
        set_type(REWARD_TYPE::MINIMAL);
    }
    virtual ~MinimalReward() = default;
    virtual Iterator begin() const override {
        return Iterator(new MinimalReward(NO_REWARD));
    }
    virtual ptr_t next() const override {
        if(reward==NO_REWARD) {
            return ptr_t(new MinimalReward(SOME_REWARD));
        } else {
            return ptr_t(new MinimalReward());
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
    virtual const char * print() const override {
        if(reward==NO_REWARD) return std::string("MinimalReward(NO_REWARD)").c_str();
        if(reward==SOME_REWARD) return std::string("MinimalReward(SOME_REWARD)").c_str();
        return std::string("MinimalReward(NONE)").c_str();
    }
    inline virtual const std::string space_descriptor() const override { return "MinimalReward"; }
};

#include "../../debug_exclude.h"

#endif /* MINIMALREWARD_H_ */
