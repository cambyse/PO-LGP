#ifndef ACTION_H_
#define ACTION_H_

#include "../util.h"

#include <ostream>

/** \brief Action objects. */
class Action: public util::NumericTypeWrapper<Action, unsigned long long int> {

public:

    enum ACTION { NULL_ACTION, UP, DOWN, LEFT, RIGHT, STAY, END_ACTION };
    static const value_t min_action = UP;
    static const value_t max_action = STAY;

    Action();
    Action(value_t val);

    static const char* action_string(const Action& a);
    const char* action_string() const;
    static Action random_action();

    friend std::ostream& operator<<(std::ostream &out, const Action& a);

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

    ActionIt();
    ActionIt(const Action& a);
    ActionIt & operator++();
    ActionIt & operator--();
    ActionIt & operator+=(const int& c);
    ActionIt & operator-=(const int& c);

    static const ActionIt first();
    static const ActionIt last();

private:
    void check_for_invalid();
};

#endif // ACTION_H_
