#ifndef DOUBLYLINKEDINSTANCE_H_
#define DOUBLYLINKEDINSTANCE_H_

#include "AbstractInstance.h"
class DoublyLinkedInstance: public AbstractInstance {
    //------------------------------------------------------------//
    //                        members                             //
    //------------------------------------------------------------//
private:
    shared_ptr_t prev_ptr, next_ptr;
    //------------------------------------------------------------//
    //                        methods                             //
    //------------------------------------------------------------//
public:
    virtual ~DoublyLinkedInstance() = default;
    static shared_ptr_t create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    virtual Iterator begin() override;
    virtual ptr_t next(const int& n = 1) const override;
    virtual ptr_t prev(const int& n = 1) const override;
    virtual shared_ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r) override;
private:
    DoublyLinkedInstance(action_ptr_t a = action_ptr_t(),
                         observation_ptr_t o = observation_ptr_t(),
                         reward_ptr_t r = reward_ptr_t(),
                         shared_ptr_t prev = AbstractInstance::create(),
                         shared_ptr_t next = AbstractInstance::create());
};

#endif /* DOUBLYLINKEDINSTANCE_H_ */
