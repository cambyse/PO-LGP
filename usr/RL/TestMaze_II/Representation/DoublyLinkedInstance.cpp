#include "DoublyLinkedInstance.h"

#define DEBUG_LEVEL 0
#include "../util/debug.h"

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::create(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r,
                                                         ptr_t before,
                                                         ptr_t after) {
    DoublyLinkedInstance * d = new DoublyLinkedInstance(a,o,r,before,after);
    ptr_t ret =  AbstractInstance::create(d); // sets self pointer
    d->prev_ptr->subscribe(d->get_self_ptr());
    d->next_ptr->subscribe(d->get_self_ptr());
    return ret;
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::create(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r) {
    return DoublyLinkedInstance::create(a,o,r,create_invalid(),create_invalid());
}

int DoublyLinkedInstance::destroy() {
    DEBUG_OUT(1, *this << " destroy");
    notify_subscribers();
    set_successor(create_invalid());
    set_predecessor(create_invalid());
    return 1;
}

// int DoublyLinkedInstance::destroy_unused_reachable() {

// }

int DoublyLinkedInstance::destroy_all_reachable() {
    if(destruction_running) {
        return 0;
    } else {
        destruction_running = true;
        notify_subscribers();
        DEBUG_OUT(1, *this << " destroy sequence");
        int destroyed = 1;
        // destroy previous
        DEBUG_OUT(2,*this << " destroy sequence --> " << prev());
        destroyed += prev()->destroy_all_reachable();
        set_predecessor(create_invalid());
        // destroy next
        DEBUG_OUT(2,*this << " destroy sequence --> " << next());
        destroyed += next()->destroy_all_reachable();
        set_successor(create_invalid());
        // finalize
        DEBUG_OUT(1, *this << " destroyed " << destroyed);
        destruction_running = false;
        return destroyed;
    }
}

// int DoublyLinkedInstance::destroy_inverse_reachable() {

// }

// int DoublyLinkedInstance::destroy_sequence() {

// }

DoublyLinkedInstance::Iterator DoublyLinkedInstance::begin() {
    DEBUG_OUT(2,*this << "->begin()");
    ptr_t first = get_self_ptr();
    ptr_t before_first = first->prev();
    while(before_first!=ptr_t()) {
        first = before_first;
        before_first = first->prev();
    }
    return Iterator(first);
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::next(const int& n) const {
    DEBUG_OUT(2,*this << "->next(" << n << ")");
    if(n<0) {
        return prev(-n);
    } else if(n==0) {
        return get_self_ptr();
    } else if(n==1) {
        return ptr_t(next_ptr);
    } else {
        return next_ptr->next(n-1);
    }
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::prev(const int& n) const {
    DEBUG_OUT(2,*this << "->prev(" << n << ")");
    if(n<0) {
        return next(-n);
    } else if(n==0) {
        return get_self_ptr();
    } else if(n==1) {
        return ptr_t(prev_ptr);
    } else {
        return prev_ptr->prev(n-1);
    }
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::append(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r) {
    DEBUG_OUT(1,*this << "->append(" << a << "," << o << "," << r << ")");
    set_successor(DoublyLinkedInstance::create(a,o,r,get_self_ptr(),create_invalid()));
    return next_ptr;
}

void DoublyLinkedInstance::set_predecessor(ptr_t pre) {
    prev_ptr->unsubscribe(get_self_ptr());
    prev_ptr = pre;
    prev_ptr->subscribe(get_self_ptr());
}

void DoublyLinkedInstance::set_successor(ptr_t suc) {
    next_ptr->unsubscribe(get_self_ptr());
    next_ptr = suc;
    next_ptr->subscribe(get_self_ptr());
}

void DoublyLinkedInstance::destruction_notification(ptr_t ins) {
    DEBUG_OUT(2,*this << " got destruction notification from " << ins);
    if(DEBUG_LEVEL>1 && ins!=next_ptr && ins!=prev_ptr) {
        DEBUG_ERROR(ins << " is neither successor nor predecessor of " << *this);
    }
    if(ins==next_ptr) {
        set_successor(create_invalid());
    }
    if(ins==prev_ptr) {
        set_predecessor(create_invalid());
    }
}

DoublyLinkedInstance::DoublyLinkedInstance(action_ptr_t a,
                                           observation_ptr_t o,
                                           reward_ptr_t r,
                                           shared_ptr_t p,
                                           shared_ptr_t n):
    AbstractInstance(a,o,r)
{
    // set predecessor
    if(p==nullptr) {
        prev_ptr = create_invalid();
    } else {
        prev_ptr = p;
    }
    // set successor
    if(n==nullptr) {
        next_ptr = create_invalid();
    } else {
        next_ptr = n;
    }
}
