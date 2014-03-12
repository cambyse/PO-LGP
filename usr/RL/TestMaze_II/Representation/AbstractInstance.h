#ifndef ABSTRACTINSTANCE_H_
#define ABSTRACTINSTANCE_H_

#include <memory>
#include <set>

#include "AbstractAction.h"
#include "AbstractObservation.h"
#include "AbstractReward.h"

#include "../util/util.h"
#include "../util/debug.h"

class AbstractInstance {
    //------------------------------------------------------------//
    //                         friends                            //
    //------------------------------------------------------------//
    // befriend derived classes so that they can access protected methods
    friend class DoublyLinkedInstance;

    //------------------------------------------------------------//
    //                    typedefs and enums                      //
    //------------------------------------------------------------//
public:
    class Iterator;
    class PointerType;
    typedef PointerType ptr_t;
    typedef AbstractAction::ptr_t action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t reward_ptr_t;
protected:
    typedef std::shared_ptr<AbstractInstance> shared_ptr_t;
    typedef std::weak_ptr<AbstractInstance> weak_ptr_t;
    typedef std::set<weak_ptr_t, std::owner_less<weak_ptr_t> > subscriber_set_t;
    enum SUBSCRIBTION_TYPE { PREDECESSOR, SUCCESSOR };
    //------------------------------------------------------------//
    //                  internal class definitions                //
    //------------------------------------------------------------//
public:
    /** \brief Wrapper for std::shared_ptr. */
    class PointerType {
    public:
        PointerType();
        PointerType(shared_ptr_t i);
        PointerType(const PointerType&) = default;
        virtual ~PointerType() final = default;
        virtual operator shared_ptr_t();
        virtual AbstractInstance & operator*() const final;
        virtual  AbstractInstance * operator->() const final;
        virtual Iterator begin() const final;
        virtual Iterator end() const final;
        virtual bool operator!=(const AbstractInstance& other) const final;
        virtual bool operator!=(const PointerType& other) const final;
        virtual bool operator==(const AbstractInstance& other) const final;
        virtual bool operator==(const PointerType& other) const final;
        virtual bool operator<(const PointerType& other) const final;
        friend inline std::ostream& operator<<(std::ostream& out, const PointerType& ptr) {
            return out << *ptr;
        }
    private:
        shared_ptr_t ptr;
    };

    /** \brief Iterator class.
     *
     * Dereferences to a pointer of the current object and calls the next()
     * function of the object to increment.*/
    class Iterator {
    public:
        Iterator(ptr_t ptr);
        /** \brief Default destructor. */
        virtual ~Iterator() final = default;
        /** \brief Dereference operator returns pointer. */
        virtual ptr_t operator*() const final;
        /** \brief Increment operator.
         *
         * Calls the next() function of the underlying object. */
        virtual Iterator & operator++() final;
        /** \brief Inequality operator.
         *
         * Checks inequality based on the underlying objects. */
        virtual bool operator!=(const Iterator& other) const final;
    private:
        /** \brief The current object. */
        ptr_t object;
    };

    //------------------------------------------------------------//
    //                         members                            //
    //------------------------------------------------------------//
public:
    const action_ptr_t action;
    const observation_ptr_t observation;
    const reward_ptr_t reward;
private:
    bool detachment_running = false;
    subscriber_set_t predecessors, successors;
    weak_ptr_t self_ptr;
    static subscriber_set_t all_instances; ///< used for memory check (only for DEBUG_LEVEL > 0)

    //------------------------------------------------------------//
    //                         methods                            //
    //------------------------------------------------------------//
public:
    virtual ~AbstractInstance();
    static ptr_t create(action_ptr_t a,
                        observation_ptr_t o,
                        reward_ptr_t r);
    static ptr_t create_invalid();
    static int memory_check();
    virtual int detach() final;
    virtual int detach_reachable() final;
    virtual int detach_all() final;
    virtual Iterator begin();
    virtual Iterator end() const final;
    virtual ptr_t next(const int& n = 1) const;
    virtual ptr_t prev(const int& n = 1) const;
    virtual bool operator!=(const AbstractInstance& other) const;
    virtual bool operator!=(const PointerType& other) const final;
    virtual bool operator==(const AbstractInstance& other) const final;
    virtual bool operator==(const PointerType& other) const final;
    virtual bool operator<(const AbstractInstance& other) const;
    virtual bool operator<(const PointerType& other) const final;
    virtual const std::string print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractInstance& a) {
        return out << a.print();
    }
    virtual ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    virtual void set_predecessor(ptr_t pre);
    virtual void set_successor(ptr_t suc);
private:
    virtual ptr_t get_self_ptr() const final;
    /** \brief Objects have to be created using create() function. */
    AbstractInstance(action_ptr_t a,
                     observation_ptr_t o,
                     reward_ptr_t r);
    /** \brief This function should be used by derived classes in their create
     * methods. It ensures that the self pointer is set. */
    static ptr_t create(AbstractInstance * pointer);
    /** \brief Subscribe to this instance to be taken into account for
     * detachment notifications. */
    virtual void subscribe(ptr_t ins, SUBSCRIBTION_TYPE t) final;
    /** \brief Unsubscribe from this instance. */
    virtual void unsubscribe(ptr_t ins, SUBSCRIBTION_TYPE t) final;
    /** \brief Subscribe to this instance to be taken into account for detach
     * callbacks. */
    virtual void detachment_notification(ptr_t ins, SUBSCRIBTION_TYPE t);
    /** \brief Notify all subscribers of pending detachment. */
    virtual void notify_subscribers() const final;
    /** \brief Set the self_ptr. This function MUST be called after creating a
     * new object. Use AbstractInstance::create(AbstractInstance * pointer). */
    virtual void set_self_ptr(shared_ptr_t p) final;
};

#include "../util/debug_exclude.h"

#endif /* ABSTRACTINSTANCE_H_ */
