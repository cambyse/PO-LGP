#ifndef ENUMERATED_REWARD_H_
#define ENUMERATED_REWARD_H_

#include "../util.h"

#include <vector>
#include <ostream>

/** \brief EnumeratedReward objects. */
class EnumeratedReward: public util::NumericTypeWrapper<EnumeratedReward, double>  {
public:
    static const std::vector<value_t> reward_vector;
    static const value_t min_reward;
    static const value_t max_reward;
    static const unsigned long reward_n;
    static const char* type_string;
    EnumeratedReward(value_t val = min_reward);
    friend std::ostream& operator<<(std::ostream &out, const EnumeratedReward& r);
    unsigned long index() const;
private:
    int add_width() const;
};

/** \brief EnumeratedRewardIt objects.
 *
 * EnumeratedRewardIt are iterators over EnumeratedReward objects. They are
 * derived from the EnumeratedReward class and can hence directly be used like
 * EnumeratedReward objects without dereferencing them. */
class EnumeratedRewardIt: public EnumeratedReward, public util::InvalidAdapter<EnumeratedRewardIt> {

public:

    // for compatibility with for( ... : ... ) constructs
    struct All {
        static EnumeratedRewardIt begin() { return EnumeratedRewardIt::first(); }
        static EnumeratedRewardIt end() { return EnumeratedRewardIt(); }
    };
    static const All all;

    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<EnumeratedRewardIt>::operator!=;
    using util::NumericTypeWrapper<EnumeratedReward, value_t>::operator!=;
    using util::InvalidAdapter<EnumeratedRewardIt>::operator==;
    using util::NumericTypeWrapper<EnumeratedReward, value_t>::operator==;

    EnumeratedRewardIt();
    /* EnumeratedRewardIt(const EnumeratedReward& r); */
    EnumeratedReward operator*() { return *this; }
    EnumeratedRewardIt & operator++();
    EnumeratedRewardIt & operator--();
    EnumeratedRewardIt & operator+=(const int& c);
    EnumeratedRewardIt & operator-=(const int& c);
    EnumeratedRewardIt operator+(const int& c) const { return EnumeratedRewardIt(*this)+=c; }
    EnumeratedRewardIt operator-(const int& c) const { return EnumeratedRewardIt(*this)-=c; }

    static const EnumeratedRewardIt first();
    static const EnumeratedRewardIt last();

private:
    unsigned long reward_index;
    EnumeratedRewardIt(const int& idx);
    void check_for_invalid();
};

#endif // ENUMERATED_REWARD_H_
