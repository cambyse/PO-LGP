#include "AbstractInstance.h"

#include "../util/util.h"
#include "../util/debug.h"

using std::string;
using std::stringstream;

AbstractInstance::AbstractInstance(action_ptr_t a, observation_ptr_t o, reward_ptr_t r):
    action(a), observation(o), reward(r), self_ptr(this)
{}

AbstractInstance::shared_ptr_t AbstractInstance::create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r) {
    AbstractInstance * i = new AbstractInstance(a,o,r);
    return i->get_self_ptr();
}

AbstractInstance::ptr_t AbstractInstance::next() const {
    return next(1);
}

AbstractInstance::ptr_t AbstractInstance::next(const int& n) const {
    if(n<0) {
        return prev(-n);
    } else if(n==0) {
        return ptr_t(get_self_ptr());
    } else {
        return next()->next(n-1);
    }
}

AbstractInstance::ptr_t AbstractInstance::prev() const {
    return prev(1);
}

AbstractInstance::ptr_t AbstractInstance::prev(const int& n) const {
    if(n<0) {
        return next(-n);
    } else if(n==0) {
        return ptr_t(get_self_ptr());
    } else {
        return this->prev()->prev(n-1);
    }
}

bool AbstractInstance::operator!=(const AbstractInstance&) const {
    DEBUG_WARNING("Using comparison operator of abstract class");
    return false;
}

bool AbstractInstance::operator<(const AbstractInstance&) const {
    DEBUG_WARNING("Using comparison operator of abstract class");
    return false;
}

const string AbstractInstance::print() const {
    stringstream s;
    s << "(" << action << "," << observation << "," << reward << ")";
    return s.str();
}

AbstractInstance::shared_ptr_t AbstractInstance::append(action_ptr_t, observation_ptr_t, reward_ptr_t) {
    DEBUG_ERROR("Cannot append to AbstractInstance");
    return shared_ptr_t(AbstractInstance().get_self_ptr());
}
