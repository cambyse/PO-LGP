#include "AbstractInstance.h"

#include "../util/debug.h"

using std::string;

AbstractInstance::ptr_t AbstractInstance::next() const {
    return ptr_t(new AbstractInstance());
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
    return string("AbstractInstance()");
}
