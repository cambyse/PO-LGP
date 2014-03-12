#include "AbstractInstance.h"

#include "../util/util.h"

#include <utility>

#define DEBUG_LEVEL 2
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

AbstractInstance::subscriber_set_t AbstractInstance::all_instances;

AbstractInstance::~AbstractInstance() {
#if DEBUG_LEVEL > 0
    auto it = all_instances.find(self_ptr);
    if(it==all_instances.end()) {
        DEBUG_ERROR(*this << " was not registered correctly");
    } else {
        all_instances.erase(it);
    }
#endif
    DEBUG_OUT(1,"Destroy " << *this);
}

AbstractInstance::ptr_t AbstractInstance::create(action_ptr_t a,
                                                 observation_ptr_t o,
                                                 reward_ptr_t r) {
    shared_ptr_t p(new AbstractInstance(a,o,r));
    p->set_self_ptr(p);
#if DEBUG_LEVEL > 0
    all_instances.insert(p);
#endif
    return ptr_t(p);
}

AbstractInstance::ptr_t AbstractInstance::create_invalid() {
    return create(action_ptr_t(),observation_ptr_t(),reward_ptr_t());
}

int AbstractInstance::memory_check() {
    return all_instances.size();
}

int AbstractInstance::detach() {
    DEBUG_OUT(1, *this << " detach");
    notify_subscribers();
    set_successor(create_invalid());
    set_predecessor(create_invalid());
    return 1;
}

int AbstractInstance::detach_reachable() {
    if(detachment_running) {
        return 0;
    } else {
        detachment_running = true;
        detach();
        DEBUG_OUT(1, *this << " detach reachable");
        int detached = 1;
        // detach previous
        DEBUG_OUT(2,*this << " detach reachable --> " << prev());
        detached += prev()->detach_reachable();
        // detach next
        DEBUG_OUT(2,*this << " detach reachable --> " << next());
        detached += next()->detach_reachable();
        // finalize
        DEBUG_OUT(1, *this << " detached " << detached);
        detachment_running = false;
        return detached;
    }
}

int AbstractInstance::detach_all() {
    if(detachment_running) {
        return 0;
    } else {
        detachment_running = true;
        notify_subscribers();
        DEBUG_OUT(1, *this << " detach all");
        int detached = 1;
        // detach previous
        DEBUG_OUT(2,*this << " detach all --> " << prev());
        detached += prev()->detach_reachable();
        set_predecessor(create_invalid());
        // detach next
        DEBUG_OUT(2,*this << " detach all --> " << next());
        detached += next()->detach_reachable();
        set_successor(create_invalid());
        // finalize
        DEBUG_OUT(1, *this << " detached " << detached);
        detachment_running = false;
        return detached;
    }
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

void AbstractInstance::subscribe(ptr_t ins, SUBSCRIBTION_TYPE t) {
    DEBUG_OUT(2,"Subscribe " << ins << " to " << *this);
    subscriber_set_t * subscribers;
    switch(t) {
    case PREDECESSOR:
        subscribers = &predecessors;
        break;
    case SUCCESSOR:
        subscribers = &successors;
        break;
    }
    subscribers->insert(weak_ptr_t(shared_ptr_t(ins)));
}

void AbstractInstance::unsubscribe(ptr_t ins, SUBSCRIBTION_TYPE t) {
    subscriber_set_t * subscribers;
    switch(t) {
    case PREDECESSOR:
        subscribers = &predecessors;
        break;
    case SUCCESSOR:
        subscribers = &successors;
        break;
    }
    auto it = subscribers->find(weak_ptr_t(shared_ptr_t(ins)));
    if(it==subscribers->end()) {
        DEBUG_ERROR("Trying to unsubscribe " << ins << " from " << *this);
    } else {
        DEBUG_OUT(2,"Unsubscribe " << ins << " from " << *this);
        subscribers->erase(it);
    }
}

void AbstractInstance::detachment_notification(ptr_t, SUBSCRIBTION_TYPE) {
    // nothing to do for AbstractInstance
    return;
}

void AbstractInstance::notify_subscribers() const {
    // Subscribers may unsubscribe in response to notification, which possibly
    // invalidates iterators. We therefore use a copy. Note that auto results in
    // a copy not a reference!
    for(auto old_subscribers : {std::make_pair(predecessors,PREDECESSOR), std::make_pair(successors,SUCCESSOR)}) {
        for(weak_ptr_t p : old_subscribers.first) {
            if(p.expired()) {
                DEBUG_DEAD_LINE;
            } else {
                DEBUG_OUT(2,*this << " sends detachment notification to " << *(p.lock()));
                p.lock()->detachment_notification(get_self_ptr(),old_subscribers.second);
            }
        }
    }
}

void AbstractInstance::set_self_ptr(shared_ptr_t p) {
    self_ptr = p;
}
