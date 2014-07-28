#include "AbstractInstance.h"

#include <utility>

#define DEBUG_STRING "AbstractInstance	("<<this<<") / "<<*this<<":	"
#define DEBUG_LEVEL 0
#include "../util/debug.h"

// Define MEMORY_CHECK to use memory_check() function (is automatically defined
// for DEBUG_LEVEL > 0).
#if DEBUG_LEVEL > 0
    #ifndef MEMORY_CHECK
        #define MEMORY_CHECK
    #endif
#endif

using std::string;
using std::stringstream;
using util::InvalidBase;
using util::INVALID;

AbstractInstance::subscriber_set_t AbstractInstance::all_instances;

//------------------------------------------------------------//
//                       PointerType                          //
//------------------------------------------------------------//

AbstractInstance::PointerType::PointerType(): ptr(create_invalid()) {}

AbstractInstance::PointerType::PointerType(shared_ptr_t i): ptr(i) {}

AbstractInstance::PointerType::PointerType(AbstractInstance * i): ptr(i) {}

AbstractInstance::PointerType::PointerType(const InvalidBase&): PointerType() {}

AbstractInstance::PointerType::operator shared_ptr_t() {
    return ptr;
}

AbstractInstance::PointerType::operator shared_const_ptr_t() {
    return ptr;
}

AbstractInstance::PointerType::operator ConstPointerType() {
    return ConstPointerType(*this);
}

AbstractInstance & AbstractInstance::PointerType::operator*() const {
    return ptr.operator*();
}

AbstractInstance * AbstractInstance::PointerType::operator->() const {
    return ptr.operator->();
}

AbstractInstance::PointerType& AbstractInstance::PointerType::operator++() {
    ptr = ptr->non_const_next();
    return *this;
}

AbstractInstance::PointerType& AbstractInstance::PointerType::operator--() {
    ptr = ptr->non_const_prev();
    return *this;
}

AbstractInstance::ConstPointerType::ConstPointerType(): ptr(create_invalid()) {}

AbstractInstance::ConstPointerType::ConstPointerType(shared_const_ptr_t i): ptr(i) {}

AbstractInstance::ConstPointerType::ConstPointerType(shared_ptr_t i): ptr(i) {}

AbstractInstance::ConstPointerType::ConstPointerType(const PointerType& p): ptr(p.ptr) {}

AbstractInstance::ConstPointerType::ConstPointerType(const InvalidBase&): ConstPointerType() {}

AbstractInstance::ConstPointerType::operator shared_const_ptr_t() {
    return ptr;
}

const AbstractInstance & AbstractInstance::ConstPointerType::operator*() const {
    return ptr.operator*();
}

const AbstractInstance * AbstractInstance::ConstPointerType::operator->() const {
    return ptr.operator->();
}

AbstractInstance::ConstPointerType& AbstractInstance::ConstPointerType::operator++() {
    ptr = ptr->const_next();
    return *this;
}

AbstractInstance::ConstPointerType& AbstractInstance::ConstPointerType::operator--() {
    ptr = ptr->const_prev();
    return *this;
}

//------------------------------------------------------------//
//                     AbstractInstance                       //
//------------------------------------------------------------//

AbstractInstance::~AbstractInstance() {
#ifdef MEMORY_CHECK
    auto it = all_instances.find(self_ptr);
    if(it==all_instances.end()) {
        DEBUG_ERROR("was not registered correctly");
    } else {
        all_instances.erase(it);
    }
#endif
    DEBUG_OUT(1,"destroy");
    delete predecessors;
    delete successors;
}

AbstractInstance::ptr_t AbstractInstance::create(action_ptr_t a,
                                                 observation_ptr_t o,
                                                 reward_ptr_t r) {
    ptr_t ptr(new AbstractInstance(a,o,r));
    ptr->set_self_ptr(ptr);
    return ptr;
}

AbstractInstance::ptr_t AbstractInstance::create_invalid() {
    return create(action_ptr_t(),observation_ptr_t(),reward_ptr_t());
}

#ifdef MEMORY_CHECK
int AbstractInstance::memory_check(bool report_entries) {
    if(DEBUG_LEVEL > 1 && report_entries) {
        for(weak_ptr_t p : all_instances) {
            if(p.expired()) {
                DEBUG_ERROR("pointer expired");
            } else {
                DEBUG_ERROR(*(p.lock()) << " not deleted");
            }
        }
    }
    return all_instances.size();
}
#else
int AbstractInstance::memory_check(bool) {
    DEBUG_ERROR("no memory check possible");
    return -1;
}
#endif

bool AbstractInstance::memory_check_request() {
#ifdef MEMORY_CHECK
    return true;
#else
    return false;
#endif
}

bool AbstractInstance::empty_memory_check() {
#ifdef MEMORY_CHECK
    return all_instances.size()==0;
#else
    //DEBUG_WARNING("No memory check possible");
    return false;
#endif
}

AbstractInstance::operator InvalidBase() const {
    if(*this==INVALID) {
        return INVALID;
    } else {
        return InvalidBase(false);
    }
}

int AbstractInstance::detach() {
    DEBUG_OUT(1,"detach");
    int detached = 0;
    if(const_prev()!=INVALID) {
        set_non_const_predecessor(INVALID);
        detached = 1;
    }
    if(const_next()!=INVALID) {
        set_non_const_successor(INVALID);
        detached = 1;
    }
    notify_subscribers();
    return detached;
}

int AbstractInstance::detach_reachable() {
    DEBUG_OUT(1,"detach reachable");
    ptr_t non_const_p = non_const_prev();
    ptr_t non_const_n = non_const_next();
    int detached = 0;
    if(const_prev()!=INVALID) {
        set_non_const_predecessor(INVALID);
        detached = 1;
    }
    if(const_next()!=INVALID) {
        set_non_const_successor(INVALID);
        detached = 1;
    }
    if(non_const_p!=INVALID) {
        DEBUG_OUT(2,"detach reachable --> " << non_const_p);
        detached += non_const_p->detach_reachable();
    }
    if(non_const_n!=INVALID) {
        DEBUG_OUT(2,"detach reachable --> " << non_const_n);
        detached += non_const_n->detach_reachable();
    }
    if(detached>0) {
        notify_subscribers();
    }
    DEBUG_OUT(1,"detached " << detached);
    return detached;
}

int AbstractInstance::detach_all() {
    DEBUG_OUT(1,"detach all");
    ptr_t p = non_const_prev();
    ptr_t n = non_const_next();
    int detached = 0;
    if(p!=INVALID) {
        set_non_const_predecessor(INVALID);
        detached += 1;
    }
    if(n!=INVALID) {
        set_non_const_successor(INVALID);
        detached += 1;
    }
    if(detached>0) {
        if(p!=INVALID) {
            DEBUG_OUT(2,"detach all --> " << p);
            detached += p->detach_all();
        }
        if(n!=INVALID) {
            DEBUG_OUT(2,"detach all --> " << n);
            detached += n->detach_all();
        }
        detached += notify_subscribers(true);
    }
    DEBUG_OUT(1,"detached " << detached);
    return detached;
}

AbstractInstance::const_ptr_t AbstractInstance::const_next(const int& n) const {
    if(n==0) {
        return get_self_ptr();
    } else {
        return INVALID;
    }
}

AbstractInstance::const_ptr_t AbstractInstance::const_prev(const int& n) const {
    if(n==0) {
        return get_self_ptr();
    } else {
        return INVALID;
    }
}

AbstractInstance::ptr_t AbstractInstance::non_const_next(const int& n) const {
    if(n==0) {
        return get_self_ptr();
    } else {
        return INVALID;
    }
}

AbstractInstance::ptr_t AbstractInstance::non_const_prev(const int& n) const {
    if(n==0) {
        return get_self_ptr();
    } else {
        return INVALID;
    }
}

AbstractInstance::const_ptr_t AbstractInstance::const_first() const {
    const_ptr_t prev = get_self_ptr();
    const_ptr_t before_prev = prev->const_prev();
    while(before_prev!=INVALID) {
        prev = before_prev;
        before_prev = prev->const_prev();
    }
    return prev;
}

AbstractInstance::const_ptr_t AbstractInstance::const_last() const {
    const_ptr_t next = get_self_ptr();
    const_ptr_t before_next = next->const_next();
    while(before_next!=INVALID) {
        next = before_next;
        before_next = next->const_next();
    }
    return next;
}

AbstractInstance::ptr_t AbstractInstance::non_const_first() const {
    ptr_t prev = get_self_ptr();
    ptr_t before_prev = prev->non_const_prev();
    while(before_prev!=INVALID) {
        prev = before_prev;
        before_prev = prev->non_const_prev();
    }
    return prev;
}

AbstractInstance::ptr_t AbstractInstance::non_const_last() const {
    ptr_t next = get_self_ptr();
    ptr_t before_next = next->non_const_next();
    while(before_next!=INVALID) {
        next = before_next;
        before_next = next->non_const_next();
    }
    return next;
}


bool AbstractInstance::operator!=(const AbstractInstance& other) const {
    if(action!=other.action) return true;
    if(observation!=other.observation) return true;
    if(reward!=other.reward) return true;
    // compare by content (risk of infinite recursion)
    // if(const_prev()!=other.const_prev()) return true;
    // if(const_next()!=other.const_next()) return true;
    // compare by adress (but only for non-INVALID)
    if((const_prev()!=INVALID || other.const_prev()!=INVALID) && shared_const_ptr_t(const_prev())!=shared_const_ptr_t(other.const_prev())) return true;
    if((const_next()!=INVALID || other.const_next()!=INVALID) && shared_const_ptr_t(const_next())!=shared_const_ptr_t(other.const_next())) return true;
    return false;
}

bool AbstractInstance::operator!=(const util::InvalidBase& other) const {
    if(other.is_invalid()) {
        if(action!=action_ptr_t()) return true;
        if(observation!=observation_ptr_t()) return true;
        if(reward!=reward_ptr_t()) return true;
        return false;
    } else {
        DEBUG_ERROR("cannot compare to non-invalid object");
        return true;
    }
}

bool AbstractInstance::operator<(const AbstractInstance& other) const {
    return self_ptr.lock()<other.self_ptr.lock();
}

const string AbstractInstance::print() const {
    stringstream s;
    s << "(" << action << "," << observation << "," << reward << ")";
    return s.str();
}

AbstractInstance::ptr_t AbstractInstance::append(action_ptr_t, observation_ptr_t, reward_ptr_t) {
    DEBUG_ERROR("cannot append to AbstractInstance");
    return self_ptr.lock();
}

void AbstractInstance::set_non_const_predecessor(ptr_t) {
    DEBUG_ERROR("cannot set predecessor");
}

void AbstractInstance::set_non_const_successor(ptr_t) {
    DEBUG_ERROR("cannot set successor");
}

void AbstractInstance::set_const_predecessor(const_ptr_t) {
    DEBUG_ERROR("cannot set predecessor");
}

void AbstractInstance::set_const_successor(const_ptr_t) {
    DEBUG_ERROR("cannot set successor");
}

AbstractInstance::ptr_t AbstractInstance::get_self_ptr() const {
    return self_ptr.lock();
}

AbstractInstance::AbstractInstance(action_ptr_t a,
                                   observation_ptr_t o,
                                   reward_ptr_t r):
    action(a), observation(o), reward(r), self_ptr(),
    predecessors(new subscriber_set_t()), successors(new subscriber_set_t())
{
    DEBUG_OUT(2,"create");
}

AbstractInstance::ptr_t AbstractInstance::create(AbstractInstance * pointer) {
    ptr_t p(pointer);
    p->set_self_ptr(p);
    return p;
}

void AbstractInstance::subscribe(ptr_t ins, SUBSCRIBTION_TYPE t) const {
    DEBUG_OUT(2,"subscribe " << ins);
    switch(t) {
    case PREDECESSOR:
        predecessors->insert(shared_ptr_t(ins));
        break;
    case SUCCESSOR:
        successors->insert(shared_ptr_t(ins));
        break;
    }
}

void AbstractInstance::unsubscribe(ptr_t ins, SUBSCRIBTION_TYPE t) const {
    subscriber_set_t * subscribers;
    switch(t) {
    case PREDECESSOR:
        subscribers = predecessors;
        break;
    case SUCCESSOR:
        subscribers = successors;
        break;
    }
    auto it = subscribers->find(weak_ptr_t(shared_ptr_t(ins)));
    if(it==subscribers->end()) {
        DEBUG_ERROR(DEBUG_STRING << "trying to unsubscribe " << ins);
    } else {
        DEBUG_OUT(2,"unsubscribe " << ins);
        subscribers->erase(it);
    }
}

void AbstractInstance::detachment_notification(const_ptr_t, SUBSCRIBTION_TYPE) {
    // nothing to do for AbstractInstance
    return;
}

int AbstractInstance::notify_subscribers(bool detach_all) const {
    int detached = 0;
    // Subscribers may unsubscribe in response to notification, which may
    // invalidate iterators --> use a copy.
    for(auto old_subscribers :
        {std::make_pair(subscriber_set_t(*predecessors),PREDECESSOR), std::make_pair(subscriber_set_t(*successors),SUCCESSOR)}
        ) {
        DEBUG_OUT(2,"notify " << old_subscribers.first.size() << (old_subscribers.second==PREDECESSOR?" predecessors":" successors"));
        for(weak_ptr_t p : old_subscribers.first) {
            if(!p.expired()) {
                DEBUG_OUT(2,"sends detachment notification to " << *(p.lock()));
                p.lock()->detachment_notification(get_self_ptr(),old_subscribers.second);
                if(detach_all) {
                    detached += p.lock()->detach_all();
                }
            }
        }
    }
    if(DEBUG_LEVEL>0) {
        for(auto old_subscribers : {predecessors,successors}) {
            for(weak_ptr_t p : *old_subscribers) {
                if(p.expired()) {
                    DEBUG_DEAD_LINE;
                }
            }
        }
    }
    return detached;
}

void AbstractInstance::set_self_ptr(ptr_t p) {
    if(!self_ptr.expired()) {
        DEBUG_ERROR("self pointer is already set");
    } else {
        self_ptr = shared_ptr_t(p);
    }
#ifdef MEMORY_CHECK
    all_instances.insert(self_ptr);
#endif
}
