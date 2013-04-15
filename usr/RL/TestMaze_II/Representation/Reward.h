/** @file This file implements the Reward and RewardIt classes. */

#ifndef REWARD_H_
#define REWARD_H_

#include "../util.h"

/** \brief Reward objects. */
class Reward: public util::NumericTypeWrapper<Reward, double>  {
public:
    static const value_t min_reward;
    static const value_t max_reward;
    static const value_t reward_increment;
    Reward(value_t val = min_reward);
};

/** \brief RewardIt objects.
 *
 * RewardIt are iterators over Reward objects. They are derived from the Reward
 * class and can hence directly be used like Reward objects without
 * dereferencing them. */
class RewardIt: public Reward  {
    
public:

    RewardIt(const Reward& r = Reward());

    RewardIt & operator++();
    RewardIt & operator--();

    bool is_valid() const;
};

#endif // REWARD_H_
