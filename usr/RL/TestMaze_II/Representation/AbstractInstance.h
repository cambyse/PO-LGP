#ifndef ABSTRACTINSTANCE_H_
#define ABSTRACTINSTANCE_H_

#include <memory>

#include "AbstractAction.h"
#include "AbstractObservation.h"
#include "AbstractReward.h"

#include "../util/util.h"
#include "../util/debug.h"

class AbstractInstance: public util::AbstractIteratableSpace<AbstractInstance> {
public:
    typedef AbstractAction::ptr_t action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t reward_ptr_t;
    typedef std::shared_ptr<AbstractInstance> shared_ptr_t;
protected:
    AbstractInstance(action_ptr_t a = action_ptr_t(),
                     observation_ptr_t o = observation_ptr_t(),
                     reward_ptr_t r = reward_ptr_t());
public:
    virtual ~AbstractInstance() = default;
    static shared_ptr_t create(action_ptr_t a = action_ptr_t(),
                               observation_ptr_t o = observation_ptr_t(),
                               reward_ptr_t r = reward_ptr_t());
    ABSTRACT_ITERATABLE_SPACE_BEGIN(AbstractInstance);
    virtual ptr_t next() const override;
    virtual ptr_t next(const int& n) const;
    virtual ptr_t prev() const;
    virtual ptr_t prev(const int& n) const;
    ABSTRACT_ITERATABLE_SPACE_NE(AbstractInstance);
    virtual bool operator!=(const AbstractInstance& other) const;
    ABSTRACT_ITERATABLE_SPACE_LT(AbstractInstance);
    virtual bool operator<(const AbstractInstance& other) const;
    virtual const std::string print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractInstance& a) {
        return out << a.print();
    }
    virtual shared_ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    shared_ptr_t get_self_ptr() const { return shared_ptr_t(self_ptr); }
    const action_ptr_t action;
    const observation_ptr_t observation;
    const reward_ptr_t reward;
protected:
    shared_ptr_t self_ptr;
};

#include "../util/debug_exclude.h"

#endif /* ABSTRACTINSTANCE_H_ */
