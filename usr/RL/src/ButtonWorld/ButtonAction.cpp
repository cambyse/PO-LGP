#include "ButtonAction.h"

#include <string>

#include <util/debug.h>

using std::vector;
using std::string;

ButtonAction::ButtonAction(int s, vector<bool> a):
    size(s), action(a)
{
    IF_DEBUG(1) {
        if(size<=0) {
            DEBUG_WARNING("Size of ButtonAction array must be larger than zero.");
        }
        if(s!=(int)a.size()) {
            DEBUG_WARNING("Array size does not match given size of ButtonAction array.");
        }
    }
    action.resize(s, false);
    set_type(ACTION_TYPE::BUTTON_ACTION);
}

ButtonAction::Iterator ButtonAction::begin() const {
    return Iterator(ptr_t(new ButtonAction(size)));
}

ButtonAction::ptr_t ButtonAction::next() const {
    vector<bool> new_action = action;
    bool carry_over = true;
    for(int idx : util::Range(size)) {
        if(new_action[idx]) {
            new_action[idx] = false;
        } else {
            new_action[idx] = true;
            carry_over = false;
            break;
        }
    }
    if(carry_over) {
        return ptr_t(new AbstractAction());
    } else {
        return ptr_t(new ButtonAction(size, new_action));
    }
}

bool ButtonAction::operator!=(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const ButtonAction *);
    return this->action != ptr->action;
}

bool ButtonAction::operator<(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const ButtonAction *);
    return this->action < ptr->action;
}

const std::string ButtonAction::print() const {
    string ret;
    if(!print_short_name) {
        ret += "ButtonAction";
    }
    ret += "(";
    bool first = true;
    for(bool b : action) {
        if(first) {
            first = false;
        } else {
            ret += ",";
        }
        ret += b?"1":"0";
    }
    ret+=")";
    return ret;
}

