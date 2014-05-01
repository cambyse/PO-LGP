#ifndef DOUBLYLINKEDINSTANCE_H_
#define DOUBLYLINKEDINSTANCE_H_

#include "AbstractInstance.h"

#include "../util/debug.h"

class DoublyLinkedInstance: public AbstractInstance {
    //------------------------------------------------------------//
    //                        members                             //
    //------------------------------------------------------------//
private:
    ptr_t prev_ptr, next_ptr;
    const_ptr_t const_prev_ptr, const_next_ptr;
    //------------------------------------------------------------//
    //                        methods                             //
    //------------------------------------------------------------//
public:
    virtual ~DoublyLinkedInstance() = default;
    static ptr_t create(action_ptr_t a,
                        observation_ptr_t o,
                        reward_ptr_t r,
                        ptr_t before,
                        ptr_t after);
    static ptr_t create(action_ptr_t a,
                        observation_ptr_t o,
                        reward_ptr_t r,
                        const_ptr_t before,
                        const_ptr_t after);
    static ptr_t create(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    static std::shared_ptr<DoublyLinkedInstance> get_shared_ptr(ptr_t ptr, bool report_error = false) {
        auto ret = std::dynamic_pointer_cast<DoublyLinkedInstance>((std::shared_ptr<AbstractInstance>)ptr);
        if(report_error&&ret==nullptr) {
            DEBUG_ERROR("Conversion failed");
        }
        return ret;
    }
    /* virtual Iterator begin() override; */
    virtual const_ptr_t const_next(const int& n = 1) const override;
    virtual const_ptr_t const_prev(const int& n = 1) const override;
    virtual ptr_t non_const_next(const int& n = 1) const override;
    virtual ptr_t non_const_prev(const int& n = 1) const override;
    virtual ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r) override;
    virtual void set_non_const_predecessor(ptr_t pre) override;
    virtual void set_non_const_successor(ptr_t suc) override;
    virtual void set_const_predecessor(const_ptr_t pre) override;
    virtual void set_const_successor(const_ptr_t suc) override;
private:
    virtual void detachment_notification(const_ptr_t ins, SUBSCRIBTION_TYPE t) override;
    DoublyLinkedInstance(action_ptr_t a,
                         observation_ptr_t o,
                         reward_ptr_t r,
                         ptr_t prev,
                         ptr_t next);
    DoublyLinkedInstance(action_ptr_t a,
                         observation_ptr_t o,
                         reward_ptr_t r,
                         const_ptr_t prev,
                         const_ptr_t next);
};

#include "../util/debug_exclude.h"

#endif /* DOUBLYLINKEDINSTANCE_H_ */
