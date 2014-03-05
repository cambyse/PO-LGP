#ifndef DOUBLYLINKEDINSTANCE_H_
#define DOUBLYLINKEDINSTANCE_H_

#include "AbstractInstance.h"

class DoublyLinkedInstance: public AbstractInstance {
private:
    DoublyLinkedInstance(action_ptr_t a,
                         observation_ptr_t o,
                         reward_ptr_t r);
public:
    virtual ~DoublyLinkedInstance() = default;
    static shared_ptr_t create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    virtual Iterator begin() const override;
    virtual ptr_t next() const override;
    virtual ptr_t prev() const override;
    virtual bool operator!=(const AbstractInstance& other) const override;
    virtual bool operator<(const AbstractInstance& other) const override;
    virtual shared_ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r) override;
private:
    shared_ptr_t next_ptr, prev_ptr;
};

#endif /* DOUBLYLINKEDINSTANCE_H_ */
