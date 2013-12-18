/**
 * @file
 * \brief This file contains an unsorted set of helper functions, templates etc.
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <math.h>
#include <float.h> // DBL_MAX
#include <QString>
#include <vector>
#include <sstream>
#include <memory> // for shared_ptr

#define DEBUG_LEVEL 0
#define DEBUG_STRING "util: "
#include "debug.h"

//========================================================//
//                       Macros                           //
//========================================================//

/** \brief Shortcut to replace for loop where the index does not play a role. */
#define repeat(n) for(unsigned long int repeat_index=0; repeat_index<(unsigned long int)n; ++repeat_index)

/** \brief Shortcut to define a print() function for a class that has defined
 * the outstream operator <<.
 *
 * Use this macro inside the cpp file to prevent inlining the function (which
 * prevents it being used in a debugger). */
#define PRINT_FROM_OSTREAM                      \
    {                                           \
        std::stringstream s;                    \
        s << *this;                             \
        return s.str().c_str();                 \
    }

/** All utility functions etc are included in the util namespace. */
namespace util {

    //========================================================//
    //              Function Declarations                     //
    //========================================================//

    /** \brief Interprete n-th argument of string as integer.
     *
     * Returns true on success and false otherwise. */
    bool arg_int(const QString& string, const int& n, int& i);

    /** \brief Interprete n-th argument of string as double.
     *
     *  Returns true on success and false otherwise. */
    bool arg_double(const QString& string, const int& n, double& d);

    /** \brief Interprete n-th argument of string as string.
     *
     *  Returns true on success and false otherwise. */
    bool arg_string(const QString& string, const int& n, QString& s);

    /** \brief Tolerance for approximate comparison. */
    double approx_equal_tolerance();

    /** \brief Return true if x compares equal to itself. */
    inline bool is_number(double x) {
        return (x == x);
    }

    /** \brief Return true if x<=DBL_MAX and x>=-DBL_MAX. */
    inline bool is_finite_number(double x) {
        return (x <= DBL_MAX && x >= -DBL_MAX);
    }

    //========================================================//
    //                      Classes                           //
    //========================================================//

    /** \brief Base class for polymorphic iteratable spaces.
     *
     * OUT OF DATE!!! An iteratable space consists of a countable set of
     * constant objects like actions in a maze (up, down, left, right, stay) or
     * the like. To define an iteratable space you can inherit from this class
     * and override the begin() end() next() and operator!=() methods. You can
     * iterate over any AbstractIteratableSpace as @code
     * for(YourIteratableSpace::ptr_t item : YourIteratableSpace()) {

         ...do things that can be done on any abstract space...

         // do a cast if you need the functionality
         // of your specific implemenation
         const YourIteratableSpace * your_item;
         your_item = dynamic_cast<const YourIteratableSpace *>(item.get());
         if(your_item!=nullptr) {

             ...do things that can only be done on your specific implementation...

         }
     }
     @endcode
     * You can use
     @code
     for(auto item : YourIteratableSpace()) { ... }
     @endcode
     * to keep the code brief. */
    template <class DerivedSpace>
    class AbstractIteratableSpace {
    public:

        class Iterator; // forward declaration for use in PointerType

        /** \brief Pointer class used as return type.
         *
         * The class uses std::shared_ptr as underlying type but defines
         * comparison operators based on the object pointed to (not the
         * adress).*/
        class PointerType {
        public:
        PointerType(): ptr(new const DerivedSpace()) {}
        PointerType(const DerivedSpace * d): ptr(d) {}
            PointerType(const PointerType&) = default;
            virtual ~PointerType() final = default;
            virtual const DerivedSpace & operator*() const final {
                return ptr.operator*();
            }
            virtual const DerivedSpace * operator->() const final {
                return ptr.operator->();
            }
            virtual Iterator begin() const final {
                return ptr->begin();
            }
            virtual Iterator end() const final {
                return ptr->end();
            }
            virtual bool operator!=(const DerivedSpace& other) const final {
                return *(this->ptr)!=other;
            }
            virtual bool operator!=(const PointerType& other) const final {
                return *(this->ptr)!=*(other.ptr);
            }
            virtual bool operator==(const DerivedSpace& other) const final {
                return !(*this!=other);
            }
            virtual bool operator==(const PointerType& other) const final {
                return !(*this!=other);
            }
            virtual bool operator<(const PointerType& other) const final {
                return *(this->ptr)<*(other.ptr);
            }
            friend inline std::ostream& operator<<(std::ostream& out, const PointerType& ptr) {
                return out << *ptr;
            }
            template < class T > std::shared_ptr<const T> get_derived(bool report_error = true) const {
                std::shared_ptr<const T> ret_ptr = std::dynamic_pointer_cast<const T>(ptr);
                if(report_error && ret_ptr == std::shared_ptr<const T>()) {
                    DEBUG_ERROR("Cast failed");
                }
                return ret_ptr;
            }
        private:
            std::shared_ptr<const DerivedSpace> ptr;
        };

        typedef PointerType ptr_t;

        /** \brief Iterator class.
         *
         * Dereferences to a pointer of the current object and calls the next()
         * function of the object to increment.*/
        class Iterator {

        public:

            Iterator(ptr_t ptr): object(ptr) {}

            Iterator(const DerivedSpace * ptr): object(ptr_t(ptr)) {}

            /** \brief Default descructor. */
            virtual ~Iterator() final = default;

            /** \brief Dereference operator returns pointer. */
            virtual ptr_t operator*() const final {
                return object;
            }

            /** \brief Increment operator.
             *
             * Calls the next() function of the underlying object. */
            virtual Iterator & operator++() final {
                object = object->next();
                return *this;
            }

            /** \brief Inequality operator.
             *
             * Checks inequality based on the underlying objects. */
            virtual bool operator!=(const Iterator& other) const final {
                return *(this->object)!=*(other.object);
            }

        private:

            /** \brief The current object. */
            ptr_t object;
        };

        /** \brief Returns Iterator to first object of the space. */
        virtual Iterator begin() const = 0;

        /** \brief Macro that should be used in derived classes to define the
         * begin function. */
#define ABSTRACT_ITERATABLE_SPACE_BEGIN(C)              \
        virtual Iterator begin() const override {       \
            return Iterator(ptr_t(new C()));            \
        }

        /** \brief Returns an invalid Iterator to stop iteration.
         *
         * A derived class must somehow implement an 'invalid' object. The
         * next() function must return a pointer to such an object if called on
         * the last element of the space and the end() function must return such
         * an object. */
        virtual Iterator end() const final {
            return Iterator(ptr_t(new const DerivedSpace()));
        }

        /** \brief Return pointer to next element in the space.
         *
         * When called on the last element in the space is should return an
         * invalid object (cf. end()). */
        virtual ptr_t next() const = 0;

        /** \brief Inequality operator.
         *
         * Used by Iterator class to determine stopping criterion. */
        virtual bool operator!=(const AbstractIteratableSpace& other) const = 0;

        virtual bool operator!=(const PointerType& other) const final {
            return *this!=*other;
        }

        /** \brief Equality operator.
         *
         * To define equality simply define the inequality operator
         * accordingly. To ensure logical consistency this operator cannot be
         * overridden, instead it returns the negation of the inequality
         * operator. */
        virtual bool operator==(const AbstractIteratableSpace& other) const final {
            return !(*this!=other);
        }

        virtual bool operator==(const PointerType& other) const final {
            return !(*this!=other);
        }

        /** \brief operator< */
        virtual bool operator<(const AbstractIteratableSpace& other) const = 0;
    };

    /** \brief Base class to make a derived class assign-compatible with a type.
     *
     * Overloads assignment and cast operators so that the class can be assigned
     * like being of type T.
     *
     * @code
     * NewClass: public AssignableTypeWrapper<NewClass,int> {
     *     NewClass(int val): AssignableTypeWrapper<NewClass,int>(val) {}
     * }
     * @endcode */
    template <class C, class T>
        class AssignableTypeWrapper {
    public:
        typedef T value_t;
    AssignableTypeWrapper(const T& val): value(val) { }
        operator T() const { return value; }
        C& operator=(const T &rhs) {
            this->value=rhs;
            return (*this);
        }
    protected:
        T value;
    };

    /** \brief Base class to provide a derived class with comparison operator.
     *
     * Inherits from util::AssignableTypeWrapper and additionally overloads
     * comparison operators so that objects of the class can be compared.
     *
     * @code
     * NewClass: public ComparableTypeWrapper<NewClass,int> {
     *     NewClass(int val): ComparableTypeWrapper<NewClass,int>(val) {}
     * }
     * @endcode */
    template <class C, class T>
        class ComparableTypeWrapper: public AssignableTypeWrapper<C,T> {
    public:
    ComparableTypeWrapper(T val): AssignableTypeWrapper<C,T>(val) { }
        bool operator==(const T &other) const { return this->value==other; }
        bool operator!=(const T &other) const { return this->value!=other; }
        bool operator<(const T &other) const { return this->value<other; }
        bool operator>(const T &other) const { return this->value>other; }
        bool operator<=(const T &other) const { return this->value<=other; }
        bool operator>=(const T &other) const { return this->value>=other; }
    };

    /** \brief Base class to provide a derived class with standard arithmetic
     * operators.
     *
     * Inherits from util::ComparableTypeWrapper and additionally overloads
     * arithmetic operators so that objects of the class can be subtracted,
     * multiplied etc.
     *
     * @code
     * NewClass: public NumericTypeWrapper<NewClass,int> {
     *     NewClass(int val): NumericTypeWrapper<NewClass,int>(val) {}
     * }
     * @endcode */
    template <class C, class T>
        class NumericTypeWrapper: public ComparableTypeWrapper<C,T> {
    public:
    NumericTypeWrapper(T val): ComparableTypeWrapper<C,T>(val) { }
        T & operator+=(const T &rhs) { return this->value+=rhs; }
        T & operator-=(const T &rhs) { return this->value-=rhs; }
        T & operator*=(const T &rhs) { return this->value*=rhs; }
        T & operator/=(const T &rhs) { return this->value/=rhs; }
    };

    /** \brief Base for objects that can be invalid.
     *
     * The globale constant util::INVALID is of this type. */
    class InvalidBase {
    public:
        /** \brief Default constructor constructs an invalid object. */
    InvalidBase(const bool& b = true): invalid(b) {}
        /** \brief Returns whether the object is invalid. */
        bool is_invalid() const { return invalid; }
    protected:
        /** \brief Holds whether the object is invalid. */
        bool invalid;
        /** \brief Invalidate this object.
         *
         * Sets this->invalid to true but keeps the rest untouched. */
        void invalidate() {
            this->invalid=true;
        }
    };

    /** \brief Base class to make a derived class comparable and assignable with
     * the globale util::INVALID object of this class.
     *
     * @code
     * NewClass: public InvalidAdapter<NewClass> {
     *     // Default constructor constructs an invalid object
     *     // if not specified differently.
     *     NewClass(): InvalidAdapter<NewClass>(true) {} // <-- redundant
     *     NewClass(): InvalidAdapter<NewClass>(false) {} // <-- non-invalid object
     * }
     * @endcode */
    template <class C>
        class InvalidAdapter: public InvalidBase {
    protected:
        /** \brief Default constructor constructs an invalid object. */
    InvalidAdapter(const bool& b = true): InvalidBase(b) {}
    public:
        /** \brief InvalidAdapter objects can be compared to InvalidBase objects.
         *
         * Returns true if both objects are valid/invalid and false if one is
         * valid but the other invalid. */
        bool operator==(const InvalidBase& i) const { return invalid==i.is_invalid(); }
        /** \brief InvalidAdapter objects can be compared to InvalidBase objects.
         *
         * Returns false if both objects are valid/invalid and true if one is
         * valid but the other invalid. */
        bool operator!=(const InvalidBase& i) const { return invalid!=i.is_invalid(); }
    private:
        /** \brief Constructing InvalidAdapter objects from derived classes is
         * not allowed. */
        InvalidAdapter(const C& c) {}
        /** \brief Assigning derived classes to invalid objects is not allowed. */
        InvalidAdapter& operator=(const C& c) {}
    };

    /** \brief Convenience class to iterate over a range of integers.
     *
     * This class allows to write something like
     * @code
     for( int i : Range(11,17) ) {
         cout << "idx = " << i << endl;
     }
     * @endcode
     * to iterate from 11 to 17 inclusively or
     * @code
     for( int i : Range(100) ) {
         cout << "idx = " << i << endl;
     }
     * @endcode
     * to iterate from 0 to 99. */
    class Range {
    public:
        /** \brief Iterator class to iterate through a Range object. */
        class RangeIt {
        public:
            RangeIt(int i, int incr);
            RangeIt & operator++();
            bool operator== (const RangeIt& other) const;
            bool operator!= (const RangeIt& other) const;
            int operator*() const { return idx; }
        private:
            int idx;
            int idx_increment;
        };

        /** \brief Iterate from \e first to \e last (inclusive) with step size
         * \e increment. */
        Range(int first, int last, int increment);

        /** \brief Iterate from \e first to \e last (inclusive) with step size
         * one. */
        Range(int first, int last);

        /** \brief Iterate from zero to \e last-1 with step size one. */
        Range(int last);

        RangeIt begin() const;
        RangeIt end() const;
    private:
        int begin_idx;
        int end_idx;
        int idx_increment;
    };

    /** \brief iRange(n) counts down from n-1 to zero. */
    class iRange: public Range {
    public:
        /** \brief Iterate from \e n-1 to zero with step size one. */
        iRange(int n);
    };

    //========================================================//
    //                  Global Variables                      //
    //========================================================//

    /** \brief Globale object of Invalid class.
     *
     * Use this object to assignment and testing of objects of derived
     * classes like:
     * @code
     * ...
     * @endcode */
    extern const InvalidBase INVALID;

    //========================================================//
    //      Function Definitions and Template Functions       //
    //========================================================//


    /** \brief Simple helper function that returns \a value clamped to the
     * interval [\a lower,\a upper].
     *
     * @param lower The lower bound for value.
     * @param upper The upper bound for value.
     * @param value Value that is to be clamped.
     * */
    template < typename T >
        T clamp(const T& lower, const T& upper, const T& value) {
        if(value<lower) {
            return lower;
        } else if(value>upper) {
            return upper;
        }
        return value;
    }

    /** \brief Select a random element from a vector. */
    template < typename T >
        T random_select(const std::vector<T> vec) {
        if(vec.size()==0) {
            DEBUG_OUT(0,"Error: Cannot draw from an empty vector");
        }
        return vec[rand()%vec.size()];
    }

    /** \brief Generic sign function.
     *
     * Returns -1 if val is less than T(0), +1 for greater, and 0 for equality. */
    template <class T>
        int sgn(T val) { return ( (val < T(0)) ? -1 : ( (val > T(0)) ? 1 : 0 ) ); }

    /** \brief Generic function to determine if c1 and c2 are approximately
     * equality.
     *
     * Returns true if the difference is smaller than the value returned by
     * util::approx_equal_tolerance(). */
    template < class C >
        bool approx_eq(const C& c1, const C& c2) { return fabs(c1-c2)<approx_equal_tolerance(); }

    /** \brief Generic function to determine if c1 is much greater than c2.
     *
     * Returns true if the c1 is greater than c2 by more than the value returned
     * by util::approx_equal_tolerance(). */
    template < class C >
        bool operator>>(const C& c1, const C& c2) { return c1>c2+approx_equal_tolerance(); }

    /** \brief Generic function to determine if c1 is much smaller than c2.
     *
     * Returns true if the c1 is less than c2 by more than the value returned by
     * util::approx_equal_tolerance(). */
    template < class C >
        bool operator<<(const C& c1, const C& c2) { return c1>c2-approx_equal_tolerance(); }

} // end namespace util

#include "debug_exclude.h"

#endif /* UTIL_H_ */
