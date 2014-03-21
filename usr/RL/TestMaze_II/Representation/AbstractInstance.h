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
    class PointerType;
    class ConstPointerType;
    typedef AbstractAction::ptr_t action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t reward_ptr_t;
    typedef PointerType ptr_t;
    typedef ConstPointerType const_ptr_t;
protected:
    enum SUBSCRIBTION_TYPE { PREDECESSOR, SUCCESSOR };
private:
    typedef std::shared_ptr<const AbstractInstance> shared_const_ptr_t;
    typedef std::shared_ptr<AbstractInstance> shared_ptr_t;
    typedef std::weak_ptr<AbstractInstance> weak_ptr_t;
    typedef std::set<weak_ptr_t, std::owner_less<weak_ptr_t> > subscriber_set_t;

    //------------------------------------------------------------//
    //                      pointer class                         //
    //------------------------------------------------------------//

    // to pass on functions to the pointed-to object
#define PASS_ON(return_t,function) \
    return_t function() const { return this->ptr->function(); }
#define BINARY_PASS_ON(return_t,function) \
    template <class T> \
    return_t function(const T& other) const { return this->ptr->function(other); }

public:
    class PointerType {
        friend class ConstPointerType;
    public:
        PointerType();
        PointerType(shared_ptr_t i);
        PointerType(AbstractInstance*);
        PointerType(const util::InvalidBase&);
        PointerType(const PointerType&) = default;
        virtual ~PointerType() final = default;
        virtual operator shared_ptr_t() final;
        virtual operator shared_const_ptr_t() final;
        virtual operator ConstPointerType() final;
        virtual AbstractInstance & operator*() const final;
        virtual AbstractInstance * operator->() const final;
        virtual PointerType& operator++() final;
        virtual PointerType& operator--() final;
        PASS_ON(,operator util::InvalidBase);
        BINARY_PASS_ON(bool,operator!=);
        BINARY_PASS_ON(bool,operator==);
        BINARY_PASS_ON(bool,operator<);
        friend inline std::ostream& operator<<(std::ostream& out, const PointerType& ptr) {
            return out << *ptr;
        }
    protected:
        shared_ptr_t ptr;
    };

    class ConstPointerType {
    public:
        ConstPointerType();
        ConstPointerType(shared_const_ptr_t i);
        ConstPointerType(shared_ptr_t i);
        ConstPointerType(const PointerType&);
        ConstPointerType(const util::InvalidBase&);
        ConstPointerType(const ConstPointerType&) = default;
        virtual ~ConstPointerType() final = default;
        virtual operator shared_const_ptr_t() final;
        virtual const AbstractInstance & operator*() const final;
        virtual const AbstractInstance * operator->() const final;
        virtual ConstPointerType& operator++() final;
        virtual ConstPointerType& operator--() final;
        PASS_ON(,operator util::InvalidBase);
        BINARY_PASS_ON(bool,operator!=);
        BINARY_PASS_ON(bool,operator==);
        BINARY_PASS_ON(bool,operator<);
        friend inline std::ostream& operator<<(std::ostream& out, const ConstPointerType& ptr) {
            return out << *ptr;
        }
    protected:
        shared_const_ptr_t ptr;
    };

#undef PASS_ON
#undef BINARY_PASS_ON

    //------------------------------------------------------------//
    //                         members                            //
    //------------------------------------------------------------//
public:
    const action_ptr_t action;
    const observation_ptr_t observation;
    const reward_ptr_t reward;
private:
    weak_ptr_t self_ptr;
    subscriber_set_t * const predecessors, * const successors;
    static subscriber_set_t all_instances; ///< used for memory check

    //------------------------------------------------------------//
    //                         methods                            //
    //------------------------------------------------------------//
public:
    virtual ~AbstractInstance();
    static ptr_t create(action_ptr_t a,
                        observation_ptr_t o,
                        reward_ptr_t r);
    static int memory_check(bool report_entries = false);
    static bool memory_check_request();
    virtual operator util::InvalidBase() const final;
    virtual int detach() final;
    virtual int detach_reachable() final;
    virtual int detach_all() final;
    virtual const_ptr_t const_next(const int& n = 1) const;
    virtual const_ptr_t const_prev(const int& n = 1) const;
    virtual ptr_t non_const_next(const int& n = 1) const;
    virtual ptr_t non_const_prev(const int& n = 1) const;
    virtual bool operator!=(const AbstractInstance& other) const;
    virtual bool operator!=(const ptr_t& other) const;
    virtual bool operator!=(const const_ptr_t& other) const;
    virtual bool operator!=(const util::InvalidBase& other) const;
    template <class T>
        bool operator==(const T& other) const { return !(*this!=other); }
    virtual bool operator<(const AbstractInstance& other) const;
    virtual const std::string print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractInstance& a) {
        return out << a.print();
    }
    virtual ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    virtual void set_non_const_predecessor(ptr_t pre);
    virtual void set_non_const_successor(ptr_t suc);
    virtual void set_const_predecessor(const_ptr_t pre);
    virtual void set_const_successor(const_ptr_t suc);
private:

    virtual ptr_t get_self_ptr() const final;

    /** \brief Objects have to be created using create() function. */
    AbstractInstance(action_ptr_t a,
                     observation_ptr_t o,
                     reward_ptr_t r);

    /** \brief This function should be used by derived classes in their create
     * methods. It ensures that the self pointer is set. */
    static ptr_t create(AbstractInstance * pointer);

    /** \brief Create a pointer to an invalid instance.
     *
     * Outside this class this can be done by assigning util::INVALID. */
    static ptr_t create_invalid();

    /** \brief Subscribe to this instance to be taken into account for
     * detachment notifications. */
    virtual void subscribe(ptr_t ins, SUBSCRIBTION_TYPE t) const final;

    /** \brief Unsubscribe from this instance. */
    virtual void unsubscribe(ptr_t ins, SUBSCRIBTION_TYPE t) const final;

    /** \brief Call this function to notify of a detachment.
     *
     * Notifies this object that ins got detached. This notification is send to
     * subscribers (PREDECESSORS or SUCCESSORS) only. This object can than reset
     * the corresponding pointers. */
    virtual void detachment_notification(const_ptr_t ins, SUBSCRIBTION_TYPE t);

    /** \brief Notify all subscribers of pending detachment.
     *
     * If detach_all argument is true, the detach_all() method is called on all
     * subscribers and the (possibly non-zero) number of resulting detachments
     * is returned.*/
    virtual int notify_subscribers(bool detach_all = false) const final;

    /** \brief Set the self_ptr. This function MUST be called after creating a
     * new object. Use AbstractInstance::create(AbstractInstance * pointer). */
    virtual void set_self_ptr(ptr_t p) final;
};

#include "../util/debug_exclude.h"

#endif /* ABSTRACTINSTANCE_H_ */
