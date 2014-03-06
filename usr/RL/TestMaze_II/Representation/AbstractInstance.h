#ifndef ABSTRACTINSTANCE_H_
#define ABSTRACTINSTANCE_H_

#include <memory>

#include "AbstractAction.h"
#include "AbstractObservation.h"
#include "AbstractReward.h"

#include "../util/util.h"
#include "../util/debug.h"

class AbstractInstance {
    //------------------------------------------------------------//
    //                         typedefs                           //
    //------------------------------------------------------------//
public:
    class Iterator;
    class PointerType;
    typedef PointerType ptr_t;
    typedef PointerType const_ptr_t;
    typedef AbstractAction::ptr_t action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t reward_ptr_t;
    typedef std::shared_ptr<AbstractInstance> shared_ptr_t;
    typedef std::shared_ptr<const AbstractInstance> const_shared_ptr_t;
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
        virtual const AbstractInstance & operator*() const final;
        virtual const AbstractInstance * operator->() const final;
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
        Iterator(shared_ptr_t ptr);
        /** \brief Default descructor. */
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
protected:
    shared_ptr_t self_ptr;

    //------------------------------------------------------------//
    //                         methods                            //
    //------------------------------------------------------------//
public:
    virtual ~AbstractInstance() = default;
    static shared_ptr_t create(action_ptr_t a = action_ptr_t(),
                               observation_ptr_t o = observation_ptr_t(),
                               reward_ptr_t r = reward_ptr_t());
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
    virtual shared_ptr_t append(action_ptr_t a, observation_ptr_t o, reward_ptr_t r);
    shared_ptr_t get_self_ptr() const { return shared_ptr_t(self_ptr); }
protected:
    AbstractInstance(action_ptr_t a = action_ptr_t(),
                     observation_ptr_t o = observation_ptr_t(),
                     reward_ptr_t r = reward_ptr_t());
};

#include "../util/debug_exclude.h"

#endif /* ABSTRACTINSTANCE_H_ */
