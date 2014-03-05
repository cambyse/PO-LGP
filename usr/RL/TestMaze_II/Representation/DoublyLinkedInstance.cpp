#include "DoublyLinkedInstance.h"

DoublyLinkedInstance::DoublyLinkedInstance(action_ptr_t a,
                                           observation_ptr_t o,
                                           reward_ptr_t r):
    AbstractInstance(a,o,r),
    next_ptr(AbstractInstance::create()),
    prev_ptr(AbstractInstance::create())
{}

DoublyLinkedInstance::shared_ptr_t DoublyLinkedInstance::create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r) {
    DoublyLinkedInstance * i = new DoublyLinkedInstance(a,o,r);
    return i->get_self_ptr();
}

DoublyLinkedInstance::Iterator DoublyLinkedInstance::begin() const {
    ptr_t first(this);
    while(first->prev()!=ptr_t(AbstractInstance::create())) {
        first = first->prev();
    }
    return Iterator(first);
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::next() const {
    return ptr_t(next_ptr);
}

DoublyLinkedInstance::ptr_t DoublyLinkedInstance::prev() const {
    return ptr_t(prev_ptr);
}

bool DoublyLinkedInstance::operator!=(const AbstractInstance& other) const {
    return this->get_self_ptr()!=other.get_self_ptr();
}

bool DoublyLinkedInstance::operator<(const AbstractInstance& other) const {
    return this->get_self_ptr()<other.get_self_ptr();
}

DoublyLinkedInstance::shared_ptr_t DoublyLinkedInstance::append(action_ptr_t a,
                                                         observation_ptr_t o,
                                                         reward_ptr_t r) {
    next_ptr = DoublyLinkedInstance(a,o,r).get_self_ptr();
    return next_ptr;
}
