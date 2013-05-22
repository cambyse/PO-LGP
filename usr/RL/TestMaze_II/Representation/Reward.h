#ifndef REWARD_H_
#define REWARD_H_

#include "../util.h"

#include <ostream>

/** \brief Reward objects. */
class Reward: public util::NumericTypeWrapper<Reward, double>  {
public:
    static const value_t min_reward;
    static const value_t max_reward;
    static const value_t reward_increment;
    static const unsigned long reward_n;
    Reward(value_t val = min_reward);
    friend std::ostream& operator<<(std::ostream &out, const Reward& r);
    unsigned long index() const { return floor((*this - min_reward)/reward_increment); }
private:
    int add_width() const;
};

/** \brief RewardIt objects.
 *
 * RewardIt are iterators over Reward objects. They are derived from the Reward
 * class and can hence directly be used like Reward objects without
 * dereferencing them. */
class RewardIt: public Reward, public util::InvalidAdapter<RewardIt> {

public:

    // for compatibility with for( ... : ... ) constructs
    struct All {
        static RewardIt begin() { return RewardIt::first(); }
        static RewardIt end() { return RewardIt(); }
    };
    static const All all;

    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<RewardIt>::operator!=;
    using util::NumericTypeWrapper<Reward, value_t>::operator!=;
    using util::InvalidAdapter<RewardIt>::operator==;
    using util::NumericTypeWrapper<Reward, value_t>::operator==;

    RewardIt();
    RewardIt(const Reward& r);
    Reward operator*() { return *this; }
    RewardIt & operator++();
    RewardIt & operator--();
    RewardIt & operator+=(const int& c);
    RewardIt & operator-=(const int& c);

    static const RewardIt first();
    static const RewardIt last();

private:
    void check_for_invalid();
};

#endif // REWARD_H_
