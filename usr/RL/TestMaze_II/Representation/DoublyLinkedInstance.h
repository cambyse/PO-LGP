#ifndef DOUBLYLINKEDINSTANCE_H_
#define DOUBLYLINKEDINSTANCE_H_

#include "AbstractInstance.h"
class DoublyLinkedInstance: public AbstractInstance {
    //------------------------------------------------------------//
    //                        members                             //
    //------------------------------------------------------------//
private:
    shared_ptr_t prev_ptr, next_ptr;
    bool prev_dismissed = false, next_dismissed = false;
    //------------------------------------------------------------//
    //                        methods                             //
    //------------------------------------------------------------//
public:
    virtual ~DoublyLinkedInstance() = default;
    static ptr_t create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    virtual void dismiss() override;
    virtual Iterator begin() override;
    virtual ptr_t next(const int& n = 1) const override;
    virtual ptr_t prev(const int& n = 1) const override;
    virtual ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r) override;
private:
    DoublyLinkedInstance(action_ptr_t a,
                         observation_ptr_t o,
                         reward_ptr_t r,
                         shared_ptr_t prev,
                         shared_ptr_t next);
};

#endif /* DOUBLYLINKEDINSTANCE_H_ */
