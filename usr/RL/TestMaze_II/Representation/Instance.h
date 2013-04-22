/** @file This file implements the Instance classe. */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "../util.h"
#include "Action.h"
#include "State.h"
#include "Reward.h"

#include <ostream>

class InstanceIt;
class ConstInstanceIt;

/** \brief Instance object.
 *
 * Instance objects are essentially triples of (Action, State, Reward)
 * representing all information associated with one time step in a reinforcement
 * learning process. Additionally every Instance stores pointers to the Instance
 * before and after, so one can iterate through time. */
class Instance {
public:
    Action action;
    State state;
    Reward reward;
    static Instance * create(const Instance&);
    static Instance * create(
        const Action& a = Action(),
        const State& s = State(),
        const Reward& r = Reward(),
        const Instance * prev = nullptr,
        const Instance * next = nullptr
        );
    ~Instance();
    bool operator<(const Instance& other) const;
    Instance * insert_instance_after  (const Action& a, const State& s, const Reward& r);
    Instance * insert_instance_before (const Action& a, const State& s, const Reward& r);
    Instance * append_instance        (const Action& a, const State& s, const Reward& r);
    Instance * prepend_instance       (const Action& a, const State& s, const Reward& r);
    const Instance * get_previous() const;
    const Instance * get_next() const;
    Instance * get_non_const_previous() const;
    Instance * get_non_const_next() const;
    InstanceIt it();
    ConstInstanceIt it() const;
    InstanceIt first();
    InstanceIt last();
    ConstInstanceIt const_first() const;
    ConstInstanceIt const_last() const;
    friend std::ostream& operator<<(std::ostream &out, const Instance& i);
protected:
    Instance * previous_instance, * next_instance;
    const Instance * const_previous_instance, * const_next_instance;
    bool previous_const, next_const;
    Instance(
        const Action& a = Action(),
        const State& s = State(),
        const Reward& r = Reward()
        );
    Instance(const Instance&);
    Instance & operator=(const Instance&);
};

/** \brief Instance iterator object. */
class InstanceIt: public util::InvalidAdapter<InstanceIt> {
public:
    InstanceIt();
    InstanceIt(Instance *);
    operator Instance*() const;
    operator ConstInstanceIt() const;
    Instance * operator->();
    InstanceIt & operator++();
    InstanceIt & operator--();
    InstanceIt & operator+=(const int& c);
    InstanceIt & operator-=(const int& c);
    InstanceIt operator+(const int& c);
    InstanceIt operator-(const int& c);
    bool operator<(const InstanceIt& other) const;
    int length_to_first() const;
    int length_to_last() const;
    friend std::ostream& operator<<(std::ostream &out, const InstanceIt& i);
protected:
    Instance * this_instance;
};

/** \brief Const Instance iterator object. */
class ConstInstanceIt: public util::InvalidAdapter<ConstInstanceIt> {
public:
    ConstInstanceIt();
    ConstInstanceIt(const Instance *);
    operator const Instance*() const;
    const Instance * operator->();
    ConstInstanceIt & operator++();
    ConstInstanceIt & operator--();
    ConstInstanceIt & operator+=(const int& c);
    ConstInstanceIt & operator-=(const int& c);
    ConstInstanceIt operator+(const int& c);
    ConstInstanceIt operator-(const int& c);
    bool operator<(const ConstInstanceIt& other) const;
    int length_to_first() const;
    int length_to_last() const;
    friend std::ostream& operator<<(std::ostream &out, const ConstInstanceIt& i);
protected:
    const Instance * this_instance;
};

//========================================================================================//
//    Old InstanceIt class

/** \brief Iterator over Instances.
 *
 * InstanceIt iterate over all possible Instance object by iterating their
 * Action, State, and Reward component. They are much like Instances and can be
 * implicitely converted to Instance objects. As opposed to what happens when
 * iterating Instance objects themselves, InstanceIt objects traverses the space
 * of possible instances, not the sequence of all instances occurred in time. */
/* class InstanceIt: public util::InvalidAdapter<InstanceIt> { */
/* public: */
/*     ActionIt action; */
/*     StateIt state; */
/*     RewardIt reward; */
/*     InstanceIt(); */
/*     InstanceIt(const ActionIt& a, const StateIt& s, const RewardIt& r); */
/*     InstanceIt & operator++(); */
/*     InstanceIt & operator--(); */
/*     InstanceIt & operator+=(const int& c); */
/*     InstanceIt & operator-=(const int& c); */
/*     static const InstanceIt first(); */
/*     static const InstanceIt last(); */
/*     operator Instance() const { return Instance(action,state,reward); } */
/*     friend std::ostream& operator<<(std::ostream &out, const InstanceIt& i); */
/* }; */

#endif // INSTANCE_H_
