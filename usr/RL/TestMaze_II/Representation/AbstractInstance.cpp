#include "AbstractInstance.h"

#include "../util/util.h"
#include "../util/debug.h"

using std::string;
using std::stringstream;

//------------------------------------------------------------//
//                       PointerType                          //
//------------------------------------------------------------//

AbstractInstance::PointerType::PointerType(): ptr(AbstractInstance::create()) {}

AbstractInstance::PointerType::PointerType(shared_ptr_t i): ptr(i) {}

const AbstractInstance & AbstractInstance::PointerType::operator*() const {
    return ptr.operator*();
}

const AbstractInstance * AbstractInstance::PointerType::operator->() const {
    return ptr.operator->();
}

AbstractInstance::Iterator AbstractInstance::PointerType::begin() const {
    return ptr->begin();
}

AbstractInstance::Iterator AbstractInstance::PointerType::end() const {
    return ptr->end();
}

bool AbstractInstance::PointerType::operator!=(const AbstractInstance& other) const {
    return *(this->ptr)!=other;
}

bool AbstractInstance::PointerType::operator!=(const PointerType& other) const {
    return *(this->ptr)!=*(other.ptr);
}

bool AbstractInstance::PointerType::operator==(const AbstractInstance& other) const {
    return !(*this!=other);
}

bool AbstractInstance::PointerType::operator==(const PointerType& other) const {
    return !(*this!=other);
}

bool AbstractInstance::PointerType::operator<(const PointerType& other) const {
    return *(this->ptr)<*(other.ptr);
}

//------------------------------------------------------------//
//                        Iterator                            //
//------------------------------------------------------------//

AbstractInstance::Iterator::Iterator(ptr_t ptr): object(ptr) {}

AbstractInstance::Iterator::Iterator(shared_ptr_t ptr): object(ptr) {}

AbstractInstance::ptr_t AbstractInstance::Iterator::operator*() const {
    return object;
}

AbstractInstance::Iterator & AbstractInstance::Iterator::operator++() {
    object = object->next();
    return *this;
}

bool AbstractInstance::Iterator::operator!=(const Iterator& other) const {
    return *(this->object)!=*(other.object);
}

//------------------------------------------------------------//
//                    AbstractInstance                        //
//------------------------------------------------------------//

AbstractInstance::shared_ptr_t AbstractInstance::create(action_ptr_t a,
                                                        observation_ptr_t o,
                                                        reward_ptr_t r) {
    AbstractInstance * i = new AbstractInstance(a,o,r);
    return i->get_self_ptr();
}

AbstractInstance::Iterator AbstractInstance::begin() {
    return Iterator(AbstractInstance::create());
}

AbstractInstance::Iterator AbstractInstance::end() const {
    return Iterator(AbstractInstance::create());
}

AbstractInstance::ptr_t AbstractInstance::next(const int& n) const {
    if(n==0) {
        return ptr_t(get_self_ptr());
    } else {
        return ptr_t(AbstractInstance::create());
    }
}

AbstractInstance::ptr_t AbstractInstance::prev(const int& n) const {
    if(n==0) {
        return ptr_t(get_self_ptr());
    } else {
        return ptr_t(AbstractInstance::create());
    }
}

bool AbstractInstance::operator!=(const AbstractInstance& other) const {
    if(action!=other.action) return true;
    if(observation!=other.observation) return true;
    if(reward!=other.reward) return true;
    return false;
}

bool AbstractInstance::operator!=(const PointerType& other) const {
    return *this!=*other;
}

bool AbstractInstance::operator==(const AbstractInstance& other) const {
    return !(*this!=other);
}

bool AbstractInstance::operator==(const PointerType& other) const {
    return !(*this!=other);
}

bool AbstractInstance::operator<(const AbstractInstance& other) const {
    return self_ptr<other.self_ptr;
}

bool AbstractInstance::operator<(const PointerType& other) const {
    return *this<*other;
}

const string AbstractInstance::print() const {
    stringstream s;
    s << "(" << action << "," << observation << "," << reward << ")";
    return s.str();
}

AbstractInstance::shared_ptr_t AbstractInstance::append(action_ptr_t, observation_ptr_t, reward_ptr_t) {
    DEBUG_ERROR("Cannot append to AbstractInstance");
    return self_ptr;
}

AbstractInstance::AbstractInstance(action_ptr_t a, observation_ptr_t o, reward_ptr_t r):
    action(a), observation(o), reward(r), self_ptr(this)
{}
