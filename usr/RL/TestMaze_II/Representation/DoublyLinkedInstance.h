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
    static ptr_t create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r, ptr_t before, ptr_t after);
    static ptr_t create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    virtual Iterator begin() override;
    virtual ptr_t next(const int& n = 1) const override;
    virtual ptr_t prev(const int& n = 1) const override;
    virtual ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r) override;
    virtual void set_predecessor(ptr_t pre) override;
    virtual void set_successor(ptr_t suc) override;
private:
    virtual void detachment_notification(ptr_t ins, SUBSCRIBTION_TYPE t) override;
    DoublyLinkedInstance(action_ptr_t a,
                         observation_ptr_t o,
                         reward_ptr_t r,
                         shared_ptr_t prev,
                         shared_ptr_t next);
};

#endif /* DOUBLYLINKEDINSTANCE_H_ */
