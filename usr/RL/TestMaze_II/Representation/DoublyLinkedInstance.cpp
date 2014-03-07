#include "DoublyLinkedInstance.h"

#define DEBUG_LEVEL 0
#include "../util/debug.h"

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::create(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r) {
    return AbstractInstance::create(new DoublyLinkedInstance(a,o,r,
                                                             create_invalid(),
                                                             create_invalid()
                                        ));
}

void DoublyLinkedInstance::dismiss() {
    DEBUG_OUT(1, *this << " dismissed");
    if(!prev_dismissed) {
        DEBUG_OUT(2,*this << " dismisses " << prev());
        prev_dismissed = true;
        prev()->dismiss();
        prev_ptr = create_invalid();
    }
    if(!next_dismissed) {
        DEBUG_OUT(2,*this << " dismisses " << next());
        next_dismissed = true;
        next()->dismiss();
        next_ptr = create_invalid();
    }
}

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
    ptr_t i = AbstractInstance::create(
        new DoublyLinkedInstance(a,o,r,get_self_ptr(),create_invalid())
        );
    next_ptr = i;
    return next_ptr;
}

DoublyLinkedInstance::DoublyLinkedInstance(action_ptr_t a,
                                           observation_ptr_t o,
                                           reward_ptr_t r,
                                           shared_ptr_t p,
                                           shared_ptr_t n):
    AbstractInstance(a,o,r)
{
    if(p==nullptr) {
        prev_ptr = create_invalid();
    } else {
        prev_ptr = p;
    }
    if(n==nullptr) {
        next_ptr = create_invalid();
    } else {
        next_ptr = n;
    }
}
