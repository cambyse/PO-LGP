/** @file This file implements the State and StateIt classes. */

#ifndef STATE_H_
#define STATE_H_

#include "../util.h"
#include "../Data.h"

#include <ostream>

/** \brief State objects. */
class State: public util::NumericTypeWrapper<State, int>  {
public:
    static const value_t min_state = 0;
    static const value_t max_state = Data::maze_x_size*Data::maze_y_size;
    State(value_t val = min_state);
    friend std::ostream& operator<<(std::ostream &out, const State& s);
    static State random_state();
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
    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<StateIt>::operator!=;
    using util::NumericTypeWrapper<State, value_t>::operator!=;
    using util::InvalidAdapter<StateIt>::operator==;
    using util::NumericTypeWrapper<State, value_t>::operator==;

    StateIt();
    StateIt(const State& s);
    StateIt & operator++();
    StateIt & operator--();
    StateIt & operator+=(const int& c);
    StateIt & operator-=(const int& c);

    static const StateIt first();
    static const StateIt last();

private:
    void check_for_invalid();
};

#endif // STATE_H_
