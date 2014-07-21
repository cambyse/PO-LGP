#ifndef SEQUENTIAL_REWARD_H_
#define SEQUENTIAL_REWARD_H_

#include "../util.h"

#include <ostream>

/** \brief SequentialReward objects. */
class SequentialReward: public util::NumericTypeWrapper<SequentialReward, double>  {
public:
    static const value_t min_reward;
    static const value_t max_reward;
    static const value_t reward_increment;
    static const unsigned long reward_n;
    static const char* type_string;
    SequentialReward(value_t val = min_reward);
    friend std::ostream& operator<<(std::ostream &out, const SequentialReward& r);
    unsigned long index() const { return floor((*this - min_reward)/reward_increment); }
private:
    int add_width() const;
};

/** \brief SequentialRewardIt objects.
 *
 * SequentialRewardIt are iterators over SequentialReward objects. They are
 * derived from the SequentialReward class and can hence directly be used like
 * SequentialReward objects without dereferencing them. */
class SequentialRewardIt: public SequentialReward, public util::InvalidAdapter<SequentialRewardIt> {

public:

    // for compatibility with for( ... : ... ) constructs
    struct All {
        static SequentialRewardIt begin() { return SequentialRewardIt::first(); }
        static SequentialRewardIt end() { return SequentialRewardIt(); }
    };
    static const All all;

    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<SequentialRewardIt>::operator!=;
    using util::NumericTypeWrapper<SequentialReward, value_t>::operator!=;
    using util::InvalidAdapter<SequentialRewardIt>::operator==;
    using util::NumericTypeWrapper<SequentialReward, value_t>::operator==;

    SequentialRewardIt();
    SequentialRewardIt(const SequentialReward& r);
    SequentialReward operator*() { return *this; }
    SequentialRewardIt & operator++();
    SequentialRewardIt & operator--();
    SequentialRewardIt & operator+=(const int& c);
    SequentialRewardIt & operator-=(const int& c);
    SequentialRewardIt operator+(const int& c) const { return SequentialRewardIt(*this)+=c; }
    SequentialRewardIt operator-(const int& c) const { return SequentialRewardIt(*this)-=c; }

    static const SequentialRewardIt first();
    static const SequentialRewardIt last();

private:
    void check_for_invalid();
};

#endif // SEQUENTIAL_REWARD_H_
