#ifndef AUGMENTEDSTATE_H_
#define AUGMENTEDSTATE_H_

#include "../util.h"

#include <ostream>

/** \brief AugmentedState objects. */
class AugmentedState: public util::NumericTypeWrapper<AugmentedState, int>  {
public:
    static const value_t min_state;
    static const value_t max_state;
    static const unsigned long state_n;
    AugmentedState(value_t val = min_state);
    friend std::ostream& operator<<(std::ostream &out, const AugmentedState& s);
    static AugmentedState random_state();
    unsigned long index() const { return *this - min_state; }
private:
    int add_width() const;
};

/** \brief AugmentedStateIt objects.
 *
 * AugmentedStateIt are iterators over AugmentedState objects. They are derived from the AugmentedState
 * class and can hence directly be used like AugmentedState object without dereferencing
 * them. */
class AugmentedStateIt: public AugmentedState, public util::InvalidAdapter<AugmentedStateIt> {

public:

    // for compatibility with for( ... : ... ) constructs
    struct All {
        static AugmentedStateIt begin() { return AugmentedStateIt::first(); }
        static AugmentedStateIt end() { return AugmentedStateIt(); }
    };
    static const All all;

    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<AugmentedStateIt>::operator!=;
    using util::NumericTypeWrapper<AugmentedState, value_t>::operator!=;
    using util::InvalidAdapter<AugmentedStateIt>::operator==;
    using util::NumericTypeWrapper<AugmentedState, value_t>::operator==;

    AugmentedStateIt();
    AugmentedStateIt(const AugmentedState& s);
    AugmentedState operator*() { return *this; }
    AugmentedStateIt & operator++();
    AugmentedStateIt & operator--();
    AugmentedStateIt & operator+=(const int& c);
    AugmentedStateIt & operator-=(const int& c);
    AugmentedStateIt operator+(const int& c) const { return AugmentedStateIt(*this)+=c; }
    AugmentedStateIt operator-(const int& c) const { return AugmentedStateIt(*this)-=c; }

    static const AugmentedStateIt first();
    static const AugmentedStateIt last();

private:
    void check_for_invalid();
};

#endif // AUGMENTEDSTATE_H_
