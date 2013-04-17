/** @file This file implements the Instance classe. */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "../util.h"
#include "Action.h"
#include "State.h"
#include "Reward.h"

#include <ostream>

/** \brief Instance object.
 *
 * Instance objects are essentially triples of (Action, State, Reward)
 * representing all information associated with one time step in a reinforcement
 * learning process. Additionally every Instance stores pointers to the Instance
 * before and after, so one can iterate through time. */
class Instance: public util::InvalidAdapter<Instance> {
public:
    Action action;
    State state;
    Reward reward;
    Instance(
        const Action& a = Action(),
        const State& s = State(),
        const Reward& r = Reward(),
        Instance * prev = nullptr,
        Instance * next = nullptr
        );
    ~Instance();
    Instance & operator++();
    Instance & operator--();
    Instance & insert_instance_after  (const Action& a, const State& s, const Reward& r);
    Instance & insert_instance_before (const Action& a, const State& s, const Reward& r);
    Instance & append_instance        (const Action& a, const State& s, const Reward& r);
    Instance & prepend_instance       (const Action& a, const State& s, const Reward& r);
    Instance first() const;
    Instance last() const;
    friend std::ostream& operator<<(std::ostream &out, const Instance& i);
protected:
    Instance * previous_instance, * next_instance;
};

/** \brief Iterator over Instances.
 *
 * InstanceIt iterate over all possible Instance object by iterating their
 * Action, State, and Reward component. They are much like Instances and can be
 * implicitely converted to Instance objects. As opposed to what happens when
 * iterating Instance objects themselves, InstanceIt objects traverses the space
 * of possible instances, not the sequence of all instances occurred in time. */
class InstanceIt: public util::InvalidAdapter<InstanceIt> {
public:
    ActionIt action;
    StateIt state;
    RewardIt reward;
    InstanceIt();
    InstanceIt(const ActionIt& a, const StateIt& s, const RewardIt& r);
    InstanceIt & operator++();
    InstanceIt & operator--();
    static const InstanceIt first();
    static const InstanceIt last();
    operator Instance() const { return Instance(action,state,reward); }
    friend std::ostream& operator<<(std::ostream &out, const InstanceIt& i);
};

#endif // INSTANCE_H_
