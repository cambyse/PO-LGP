#ifndef DOUBLYLINKEDINSTANCE_H_
#define DOUBLYLINKEDINSTANCE_H_

#include "AbstractInstance.h"
class DoublyLinkedInstance: public AbstractInstance {
    //------------------------------------------------------------//
    //                        members                             //
    //------------------------------------------------------------//
private:
    shared_ptr_t prev_ptr, next_ptr;
    bool destruction_running = false;
    //------------------------------------------------------------//
    //                        methods                             //
    //------------------------------------------------------------//
public:
    virtual ~DoublyLinkedInstance() = default;
    static ptr_t create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r, ptr_t before, ptr_t after);
    static ptr_t create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    virtual int destroy() override;
    /* virtual int destroy_unused_reachable() override; */
    virtual int destroy_all_reachable() override;
    /* virtual int destroy_inverse_reachable() override; */
    /* virtual int destroy_sequence() override; */
    virtual Iterator begin() override;
    virtual ptr_t next(const int& n = 1) const override;
    virtual ptr_t prev(const int& n = 1) const override;
    virtual ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r) override;
    virtual void set_predecessor(ptr_t pre) override;
    virtual void set_successor(ptr_t suc) override;
private:
    virtual void destruction_notification(ptr_t ins);
    DoublyLinkedInstance(action_ptr_t a,
                         observation_ptr_t o,
                         reward_ptr_t r,
                         shared_ptr_t prev,
                         shared_ptr_t next);
};

#endif /* DOUBLYLINKEDINSTANCE_H_ */
