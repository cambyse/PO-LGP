#ifndef MINIMALREWARD_H_
#define MINIMALREWARD_H_

#include "../../AbstractReward.h"

class MinimalReward: public AbstractReward {
public:
    enum REWARD { ONE, TWO, NONE } reward;
    MazeReward(REWARD o = NONE) {
        reward = o;
        set_type(REWARD_TYPE::MINIMAL);
    }
    virtual ~MinimalReward() = default;
    virtual Iterator begin() const override {
        return Iterator(new MazeReward(ONE));
    }
    virtual ptr_t next() const override {
        if(reward==ONE) return ptr_t(new MinimalReward(TWO));
        return end();
    }
    virtual bool operator!=(const AbstractReward &other) const override {
        return reward!=other.reward;
    }
    virtual bool operator<(const AbstractReward &other) const override {
        return reward<other.reward;
    }
    virtual const char * print() const override {
        if(reward==ONE) return std::string("MinimalReward(ONE)").c_str();
        if(reward==TWO) return std::string("MinimalReward(TWO)").c_str();
        return std::string("MinimalReward(NONE)").c_str();
    }
    inline virtual const std::string space_descriptor() const override { return "MinimalReward"; }
};

#endif /* MINIMALREWARD_H_ */
