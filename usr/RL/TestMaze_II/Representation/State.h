#ifndef STATE_H_
#define STATE_H_

#include "../util.h"
#include "../Data.h"

#include <ostream>

/** \brief State objects. */
class State: public util::NumericTypeWrapper<State, int>  {
public:
    static const value_t min_state;
    static const value_t max_state;
    static const unsigned long state_n;
    State(value_t val = min_state);
    friend std::ostream& operator<<(std::ostream &out, const State& s);
    static State random_state();
    unsigned long index() const { return *this - min_state; }
private:
    int add_width() const;
};

/** \brief StateIt objects.
 *
 * StateIt are iterators over State objects. They are derived from the State
 * class and can hence directly be used like State object without dereferencing
 * them. */
class StateIt: public State, public util::InvalidAdapter<StateIt> {

public:

    // for compatibility with for( ... : ... ) constructs
    struct All {
        static StateIt begin() { return StateIt::first(); }
        static StateIt end() { return StateIt(); }
    };
    static const All all;

    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<StateIt>::operator!=;
    using util::NumericTypeWrapper<State, value_t>::operator!=;
    using util::InvalidAdapter<StateIt>::operator==;
    using util::NumericTypeWrapper<State, value_t>::operator==;

    StateIt();
    StateIt(const State& s);
    State operator*() { return *this; }
    StateIt & operator++();
    StateIt & operator--();
    StateIt & operator+=(const int& c);
    StateIt & operator-=(const int& c);
    StateIt operator+(const int& c) const { return StateIt(*this)+=c; }
    StateIt operator-(const int& c) const { return StateIt(*this)-=c; }

    static const StateIt first();
    static const StateIt last();

private:
    void check_for_invalid();
};

#endif // STATE_H_
