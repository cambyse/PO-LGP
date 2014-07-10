#include "DoublyLinkedInstance.h"

#define DEBUG_STRING "DoublyLinkedInstance	("<<this<<") / "<<*this<<":	"
#define DEBUG_LEVEL 0
#include "../util/debug.h"

using util::INVALID;

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::create(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r,
                                                         ptr_t before,
                                                         ptr_t after) {
    DoublyLinkedInstance * d = new DoublyLinkedInstance(a,o,r,before,after);
    ptr_t ret =  AbstractInstance::create(d); // sets self pointer
    d->const_prev()->subscribe(d->get_self_ptr(),SUCCESSOR);
    d->const_next()->subscribe(d->get_self_ptr(),PREDECESSOR);
    return ret;
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::create(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r,
                                                         const_ptr_t before,
                                                         const_ptr_t after) {
    DoublyLinkedInstance * d = new DoublyLinkedInstance(a,o,r,before,after);
    ptr_t ret =  AbstractInstance::create(d); // sets self pointer
    d->const_prev()->subscribe(d->get_self_ptr(),SUCCESSOR);
    d->const_next()->subscribe(d->get_self_ptr(),PREDECESSOR);
    return ret;
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::create(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r) {
    return DoublyLinkedInstance::create(a,o,r,ptr_t(INVALID),ptr_t(INVALID));
}

DoublyLinkedInstance::const_ptr_t DoublyLinkedInstance::const_next(const int& n) const {
    if(n<0) {
        return const_prev(-n);
    } else if(n==0) {
        return get_self_ptr();
    } else if(n==1) {
        return const_next_ptr;
    } else {
        return const_next()->const_next(n-1);
    }
}

DoublyLinkedInstance::const_ptr_t DoublyLinkedInstance::const_prev(const int& n) const {
    if(n<0) {
        return const_next(-n);
    } else if(n==0) {
        return get_self_ptr();
    } else if(n==1) {
        return const_prev_ptr;
    } else {
        return const_prev()->const_prev(n-1);
    }
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::non_const_next(const int& n) const {
    if(n<0) {
        return non_const_prev(-n);
    } else if(n==0) {
        return get_self_ptr();
    } else if(n==1) {
        return next_ptr;
    } else {
        return non_const_next()->non_const_next(n-1);
    }
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::non_const_prev(const int& n) const {
    if(n<0) {
        return non_const_next(-n);
    } else if(n==0) {
        return get_self_ptr();
    } else if(n==1) {
        return prev_ptr;
    } else {
        return non_const_prev()->non_const_prev(n-1);
    }
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::append(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r) {
    DEBUG_OUT(1,"append(" << a << "," << o << "," << r << ")");
    set_non_const_successor(DoublyLinkedInstance::create(a,o,r,get_self_ptr(),INVALID));
    return non_const_next();
}

void DoublyLinkedInstance::set_non_const_predecessor(ptr_t pre) {
    const_prev()->unsubscribe(get_self_ptr(),SUCCESSOR);
    prev_ptr = pre;
    const_prev_ptr = pre;
    const_prev()->subscribe(get_self_ptr(),SUCCESSOR);
}

void DoublyLinkedInstance::set_non_const_successor(ptr_t suc) {
    const_next()->unsubscribe(get_self_ptr(),PREDECESSOR);
    next_ptr = suc;
    const_next_ptr = suc;
    const_next()->subscribe(get_self_ptr(),PREDECESSOR);
}

void DoublyLinkedInstance::set_const_predecessor(const_ptr_t pre) {
    const_prev()->unsubscribe(get_self_ptr(),SUCCESSOR);
    prev_ptr = INVALID;
    const_prev_ptr = pre;
    const_prev()->subscribe(get_self_ptr(),SUCCESSOR);
}

void DoublyLinkedInstance::set_const_successor(const_ptr_t suc) {
    const_next()->unsubscribe(get_self_ptr(),PREDECESSOR);
    next_ptr = INVALID;
    const_next_ptr = suc;
    const_next()->subscribe(get_self_ptr(),PREDECESSOR);
}

void DoublyLinkedInstance::detachment_notification(const_ptr_t ins, SUBSCRIBTION_TYPE t) {
    DEBUG_OUT(2,"got detachment notification from " << ins);
    switch(t) {
    case PREDECESSOR:
        if(DEBUG_LEVEL>1 && ins!=const_next()) {
            DEBUG_ERROR(ins << " is not successor (should be " << const_next() << ")");
        } else if(ins==const_next()) {
            set_non_const_successor(INVALID);
        }
        break;
    case SUCCESSOR:
        if(DEBUG_LEVEL>1 && ins!=const_prev()) {
            DEBUG_ERROR(ins << " is not predecessor (should be " << const_prev() << ")");
        } else if(ins==const_prev()) {
            set_non_const_predecessor(INVALID);
        }
    }
}

DoublyLinkedInstance::DoublyLinkedInstance(action_ptr_t a,
                                           observation_ptr_t o,
                                           reward_ptr_t r,
                                           ptr_t p,
                                           ptr_t n):
    AbstractInstance(a,o,r), prev_ptr(p), next_ptr(n), const_prev_ptr(p), const_next_ptr(n)
{}

DoublyLinkedInstance::DoublyLinkedInstance(action_ptr_t a,
                                           observation_ptr_t o,
                                           reward_ptr_t r,
                                           const_ptr_t p,
                                           const_ptr_t n):
    AbstractInstance(a,o,r),
    prev_ptr(INVALID), next_ptr(INVALID), const_prev_ptr(p), const_next_ptr(n)
{}
