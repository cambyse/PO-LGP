/** @file This file implements the Action and ActionIt classes. */

#ifndef ACTION_H_
#define ACTION_H_

#include "util.h"

/** \brief Action objects. */
class Action: public util::NumericTypeWrapper<Action, unsigned long long int>  {

public:

    enum ACTION { NULL_ACTION, UP, DOWN, LEFT, RIGHT, STAY, END_ACTION };
    
    Action(value_t val = NULL_ACTION);

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
class ActionIt: public Action  {
    
public:

    ActionIt(const Action& a = Action(NULL_ACTION+1));

    ActionIt & operator++();
    ActionIt & operator--();

    bool is_valid() const;
};

#endif // ACTION_H_
