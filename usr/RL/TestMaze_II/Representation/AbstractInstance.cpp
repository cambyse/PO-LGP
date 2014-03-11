#include "AbstractInstance.h"

#include "../util/util.h"

#define DEBUG_LEVEL 0
#include "../util/debug.h"

using std::string;
using std::stringstream;

//------------------------------------------------------------//
//                       PointerType                          //
//------------------------------------------------------------//

AbstractInstance::PointerType::PointerType(): ptr(AbstractInstance::create_invalid()) {}

AbstractInstance::PointerType::PointerType(shared_ptr_t i): ptr(i) {}

AbstractInstance::PointerType::operator shared_ptr_t() {
    return ptr;
}

AbstractInstance & AbstractInstance::PointerType::operator*() const {
    return ptr.operator*();
}

AbstractInstance * AbstractInstance::PointerType::operator->() const {
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

AbstractInstance::~AbstractInstance() {
    DEBUG_OUT(1,"Destroy " << *this);
}

AbstractInstance::ptr_t AbstractInstance::create(action_ptr_t a,
                                                 observation_ptr_t o,
                                                 reward_ptr_t r) {
    shared_ptr_t p(new AbstractInstance(a,o,r));
    p->set_self_ptr(p);
    return ptr_t(p);
}

AbstractInstance::ptr_t AbstractInstance::create_invalid() {
    return create(action_ptr_t(),observation_ptr_t(),reward_ptr_t());
}

int AbstractInstance::destroy() {
    DEBUG_OUT(1, *this << " destroy");
    notify_subscribers();
    return 0;
}

int AbstractInstance::destroy_unused_reachable() {
    DEBUG_OUT(1, *this << " destroy_unused_reachable");
    notify_subscribers();
    return 0;
}

int AbstractInstance::destroy_all_reachable() {
    DEBUG_OUT(1, *this << " destroy_all_reachable");
    notify_subscribers();
    return 0;
}

int AbstractInstance::destroy_inverse_reachable() {
    DEBUG_OUT(1, *this << " destroy_inverse_reachable");
    notify_subscribers();
    return 0;
}

int AbstractInstance::destroy_sequence() {
    DEBUG_OUT(1, *this << " destroy_sequence");
    notify_subscribers();
    return 0;
}

AbstractInstance::Iterator AbstractInstance::begin() {
    return Iterator(AbstractInstance::create_invalid());
}

AbstractInstance::Iterator AbstractInstance::end() const {
    return Iterator(create_invalid());
}

AbstractInstance::ptr_t AbstractInstance::next(const int& n) const {
    if(n==0) {
        return ptr_t(get_self_ptr());
    } else {
        return ptr_t(AbstractInstance::create_invalid());
    }
}

AbstractInstance::ptr_t AbstractInstance::prev(const int& n) const {
    if(n==0) {
        return ptr_t(get_self_ptr());
    } else {
        return ptr_t(AbstractInstance::create_invalid());
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
    return self_ptr.lock()<other.self_ptr.lock();
}

bool AbstractInstance::operator<(const PointerType& other) const {
    return *this<*other;
}

const string AbstractInstance::print() const {
    stringstream s;
    s << "(" << action << "," << observation << "," << reward << ")";
    return s.str();
}

AbstractInstance::ptr_t AbstractInstance::append(action_ptr_t, observation_ptr_t, reward_ptr_t) {
    DEBUG_ERROR("Cannot append to AbstractInstance");
    return self_ptr.lock();
}

void AbstractInstance::set_predecessor(ptr_t) {
    DEBUG_ERROR("Cannot set predecessor of " << *this);
}

void AbstractInstance::set_successor(ptr_t) {
    DEBUG_ERROR("Cannot set successor of " << *this);
}

AbstractInstance::ptr_t AbstractInstance::get_self_ptr() const {
    return ptr_t(self_ptr.lock());
}

AbstractInstance::AbstractInstance(action_ptr_t a,
                                   observation_ptr_t o,
                                   reward_ptr_t r):
    action(a), observation(o), reward(r), self_ptr()
{}

AbstractInstance::ptr_t AbstractInstance::create(AbstractInstance * pointer) {
    shared_ptr_t p(pointer);
    p->set_self_ptr(p);
    return ptr_t(p);
}

void AbstractInstance::subscribe(ptr_t ins) {
    DEBUG_OUT(2,"Subscribe " << ins << " to " << *this);
    subscribers.insert(weak_ptr_t(shared_ptr_t(ins)));
}

void AbstractInstance::unsubscribe(ptr_t ins) {
    auto it = subscribers.find(weak_ptr_t(shared_ptr_t(ins)));
    if(it==subscribers.end()) {
        DEBUG_ERROR("Trying to unsubscribe " << ins << " from " << *this);
    } else {
        DEBUG_OUT(2,"Unsubscribe " << ins << " from " << *this);
        subscribers.erase(it);
    }
}

void AbstractInstance::destruction_notification(ptr_t) {
    // nothing to do for AbstractInstance
    return;
}

void AbstractInstance::notify_subscribers() const {
    // Subscribers may unsubscribe in response to notification, which possibly
    // invalidates iterators. We therefore use a copy.
    subscriber_set_t old_subscribers = subscribers;
    for(weak_ptr_t p : old_subscribers) {
        if(p.expired()) {
            DEBUG_DEAD_LINE;
        } else {
            DEBUG_OUT(2,*this << " sends destruction notification to " << *(p.lock()));
            p.lock()->destruction_notification(get_self_ptr());
        }
    }
}

void AbstractInstance::set_self_ptr(shared_ptr_t p) {
    self_ptr = p;
}
