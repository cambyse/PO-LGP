/** @file This file implements the Action and ActionIt classes. */

#ifndef ACTION_H_
#define ACTION_H_

#include "util.h"

/** \brief Action objects. */
class Action: public util::NumericTypeWrapper<Action, unsigned long long int> {

public:

    enum ACTION { NULL_ACTION, UP, DOWN, LEFT, RIGHT, STAY, END_ACTION };

    Action();
    Action(value_t val);

    static const char* action_string(const Action& a);
    const char* action_string() const;

protected:

    static const char* action_strings[END_ACTION];

};

/** \brief ActionIt objects.
 *
 * ActionIt are iterators over Action objects. They are derived from the Action
 * class and can hence directly be used like Action object without dereferencing
 * them. */
class ActionIt: public Action, public util::InvalidAdapter<ActionIt> {

public:

    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<ActionIt>::operator!=;
    using util::NumericTypeWrapper<Action, value_t>::operator!=;
    using util::InvalidAdapter<ActionIt>::operator==;
    using util::NumericTypeWrapper<Action, value_t>::operator==;

    ActionIt(const Action& a = Action(NULL_ACTION+1));
    ActionIt & operator++();
    ActionIt & operator--();

private:
    void check_for_invalid();
};

#endif // ACTION_H_
