#include "DoublyLinkedInstance.h"

#include "../util/debug.h"

DoublyLinkedInstance::shared_ptr_t DoublyLinkedInstance::create(action_ptr_t a,
                                                                observation_ptr_t o,
                                                                reward_ptr_t r) {
    DoublyLinkedInstance * i = new DoublyLinkedInstance(a,o,r);
    return i->get_self_ptr();
}

DoublyLinkedInstance::Iterator DoublyLinkedInstance::begin() {
    ptr_t first(self_ptr);
    while(first->prev()!=ptr_t()) {
        DEBUG_OUT(0,first);
        DEBUG_OUT(0,ptr_t());
        first = first->prev();
    }
    return Iterator(first);
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::next(const int& n) const {
    if(n<0) {
        return prev(-n);
    } else if(n==0) {
        return self_ptr;
    } else if(n==1) {
        return ptr_t(next_ptr);
    } else {
        return next_ptr->next(n-1);
    }
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::prev(const int& n) const {
    if(n<0) {
        return next(-n);
    } else if(n==0) {
        return self_ptr;
    } else if(n==1) {
        return ptr_t(prev_ptr);
    } else {
        return prev_ptr->prev(n-1);
    }
}

DoublyLinkedInstance::shared_ptr_t DoublyLinkedInstance::append(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r) {
    next_ptr = DoublyLinkedInstance::create(a,o,r);
    return next_ptr;
}

DoublyLinkedInstance::DoublyLinkedInstance(action_ptr_t a,
                                           observation_ptr_t o,
                                           reward_ptr_t r,
                                           shared_ptr_t p,
                                           shared_ptr_t n):
    AbstractInstance(a,o,r),
    prev_ptr(p),
    next_ptr(n)
{}
