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
