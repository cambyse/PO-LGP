#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "../util.h"
#include "../Config.h"
#include "Action.h"
#include "State.h"
#include "Reward.h"

#include <ostream>
#include <vector>

class InstanceIt;
class ConstInstanceIt;

/** \example Instance_Example.cpp This is an an example of how to use the
 * Instance class. It shows different ways of generating sequences of instances
 * and iterating through them. */

/** \brief Instance object.
 *
 * Instance objects are essentially triples of (Action, State, Reward)
 * representing all information associated with one time step in a reinforcement
 * learning process. Additionally every Instance stores pointers to the Instance
 * before and after, so one can iterate through time. */
class Instance {

public:

    USE_CONFIG_TYPEDEFS;

    friend class InstanceIt;
    friend class ConstInstanceIt;
    Action action;
    State state;
    Reward reward;

    // for compatibility with for( ... : ... ) constructs
    class All {
    public:
        All(Instance * i);
        InstanceIt begin();
        InstanceIt end();
    protected:
        Instance * instance;
    } all;
    class ConstAll {
    public:
        ConstAll(const Instance * i);
        ConstInstanceIt begin();
        ConstInstanceIt end();
    protected:
        const Instance * instance;
    } const_all;

    static Instance * create(
        const Action& a = Action(),
        const State& s = State(),
        const Reward& r = Reward(),
        const Instance * prev = nullptr,
        const Instance * next = nullptr
        );
    /* static Instance * create(const Instance&); */
    ~Instance();
    bool operator<(const Instance& other) const;
    Instance * insert_instance_after  (const Action& a, const State& s, const Reward& r);
    Instance * insert_instance_before (const Action& a, const State& s, const Reward& r);
    Instance * append_instance        (const Action& a, const State& s, const Reward& r);
    Instance * prepend_instance       (const Action& a, const State& s, const Reward& r);
    InstanceIt it();
    ConstInstanceIt const_it() const;
    InstanceIt first();
    InstanceIt last();
    ConstInstanceIt const_first() const;
    ConstInstanceIt const_last() const;
    void set_container();
    void unset_container();
    friend std::ostream& operator<<(std::ostream &out, const Instance& i);
    const char* print();
    void set_const_previous(const Instance * prev) { const_previous_instance = prev; }
    void set_const_next(const Instance * next) { const_next_instance = next; }

    /** \brief This function performs a number of benchmark tests.
     *
     * Running this function shows some statistics on the Instance/InstanceIt
     * class. By redirecting the output to a text file and using a gnuplot
     * script it is for instance possible to generate graphs like the following:
     * \image html Instance_Initialization.jpg
     * \image html Instance_Set-Unset_Container.jpg
     * \image html Instance_Iteration.jpg
     * \image html Instance_Random_Access.jpg
     * */
    static void check_performance_and_memory(bool memory = false);

protected:

    typedef std::vector<Instance*> container_t;
    Instance * previous_instance, * next_instance;
    const Instance * const_previous_instance, * const_next_instance;
    container_t * container;
    idx_t container_idx;

    const Instance * get_previous() const;
    const Instance * get_next() const;
    Instance * get_non_const_previous() const;
    Instance * get_non_const_next() const;
    void  unset_container_elements();
    void fill_container(Instance *);
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
    USE_CONFIG_TYPEDEFS;
    InstanceIt();
    InstanceIt(Instance *);
    operator Instance*() const;
    operator ConstInstanceIt() const;
    Instance * operator->();
    InstanceIt & operator++();
    InstanceIt & operator--();
    InstanceIt & operator+=(const int& c);
    InstanceIt & operator-=(const int& c);
    InstanceIt operator+(const int& c) const;
    InstanceIt operator-(const int& c) const;
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
    USE_CONFIG_TYPEDEFS;
    ConstInstanceIt();
    ConstInstanceIt(const Instance *);
    operator const Instance*() const;
    const Instance * operator->();
    ConstInstanceIt & operator++();
    ConstInstanceIt & operator--();
    ConstInstanceIt & operator+=(const int& c);
    ConstInstanceIt & operator-=(const int& c);
    ConstInstanceIt operator+(const int& c) const;
    ConstInstanceIt operator-(const int& c) const;
    bool operator<(const ConstInstanceIt& other) const;
    int length_to_first() const;
    int length_to_last() const;
    friend std::ostream& operator<<(std::ostream &out, const ConstInstanceIt& i);
protected:
    const Instance * this_instance;
};

#endif /* INSTANCE_H_ */
