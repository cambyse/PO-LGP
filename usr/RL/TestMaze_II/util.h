/**
 * @file
 * \brief This file contains an unsorted set of helper functions, templates etc.
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <math.h>
#include <QString>

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

    /** \brief Generic function that returns the smaller of c1 and c2 (c2 for
     * equality). */
    template < class C >
        C min(const C& c1, const C& c2) { return c1<c2 ? c1 : c2; }

    /** \brief Generic function that returns the greater of c1 and c2 (c2 for
     * equality). */
    template < class C >
        C max(const C& c1, const C& c2) { return c1>c2 ? c1 : c2; }

    /** \brief Generic sign function.
     *
     * Returns -1 if val is less than T(0), +1 for greater, and 0 for equality. */
    template <class T>
        int sgn(T val) { return ( (val < T(0)) ? -1 : ( (val > T(0)) ? 1 : 0 ) ); }

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
    AssignableTypeWrapper(T val): value(val) { }
        operator T() { return value; }
        T& operator=(const C &rhs) { return this->value=rhs.value; }
        T& operator=(const T &rhs) { return this->value=rhs; }
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
        bool operator==(const C &other) const { return this->value==other.value; }
        bool operator==(const T &other) const { return this->value==other; }
        bool operator!=(const C &other) const { return this->value!=other.value; }
        bool operator!=(const T &other) const { return this->value!=other; }
        bool operator<(const C &other) const { return this->value<other.value; }
        bool operator<(const T &other) const { return this->value<other; }
        bool operator>(const C &other) const { return this->value>other.value; }
        bool operator>(const T &other) const { return this->value>other; }
        bool operator<=(const C &other) const { return this->value<=other.value; }
        bool operator<=(const T &other) const { return this->value<=other; }
        bool operator>=(const C &other) const { return this->value>=other.value; }
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
        T & operator+=(const C &rhs) { return this->value+=rhs.value; }
        T & operator+=(const T &rhs) { return this->value+=rhs; }
        T & operator-=(const C &rhs) { return this->value-=rhs.value; }
        T & operator-=(const T &rhs) { return this->value-=rhs; }
        T & operator*=(const C &rhs) { return this->value*=rhs.value; }
        T & operator*=(const T &rhs) { return this->value*=rhs; }
        T & operator/=(const C &rhs) { return this->value/=rhs.value; }
        T & operator/=(const T &rhs) { return this->value/=rhs; }
        const T operator+(const C &rhs) const { return this->value+rhs.value; }
        const T operator+(const T &rhs) const { return this->value+rhs; }
        const T operator-(const C &rhs) const { return this->value-rhs.value; }
        const T operator-(const T &rhs) const { return this->value-rhs; }
        const T operator*(const C &rhs) const { return this->value*rhs.value; }
        const T operator*(const T &rhs) const { return this->value*rhs; }
        const T operator/(const C &rhs) const { return this->value/rhs.value; }
        const T operator/(const T &rhs) const { return this->value/rhs; }
    };

    //========================================================//
    //              Function Definitions                      //
    //========================================================//

    /** \brief Tolerance for approximate comparison. */
    double approx_equal_tolerance();

    /** \brief Generic function to determine if c1 and c2 are approximately
     * equality.
     *
     * Returns true if the difference is smaller than the value returned by
     * util::approx_equal_tolerance(). */
    template < class C >
        bool approx(const C& c1, const C& c2) { return fabs(c1-c2)<approx_equal_tolerance(); }

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

#endif /* UTIL_H_ */
