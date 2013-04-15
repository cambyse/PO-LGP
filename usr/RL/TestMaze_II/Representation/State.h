/** @file This file implements the State and StateIt classes. */

#ifndef STATE_H_
#define STATE_H_

#include "../util.h"

/** \brief State objects. */
class State: public util::NumericTypeWrapper<State, unsigned long long int>  {
public:
    static const value_t min_state = 1;
    static const value_t max_state = 100;
    State(value_t val = min_state);
};

/** \brief StateIt objects.
 *
 * StateIt are iterators over State objects. They are derived from the State
 * class and can hence directly be used like State object without dereferencing
 * them. */
class StateIt: public State  {
    
public:

    StateIt(const State& a = State());

    StateIt & operator++();
    StateIt & operator--();

    bool is_valid() const;
};

#endif // STATE_H_
