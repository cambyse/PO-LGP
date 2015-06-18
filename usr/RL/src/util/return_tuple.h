#ifndef TUPLERETURN_H_
#define TUPLERETURN_H_

/** \file return_tuple.h
 *
 * This files defines convenience macros and function for assigning values to
 * tuples of variables.
 *
 * All the examples below result in @code i==1 @endcode and @code d==1.1
 * @endcode being true. You may create variables and then assign values to them
 * as a tuple:
 * @code
 * int i; double d;
 * t(i,d) = std::make_tuple(1, 1.1);
 * @endcode
 *
 * You can also use a macro to declare a the variables along with a tuple of
 * references:
 * @code
 * TN(tt, int, i, double, d);
 * tt = std::make_tuple(1, 1.1);
 * @endcode
 *
 * The same macro can be used to perform declaration and assignment in one step:
 * @code
 * TN(tt, int, i, double, d) = std::make_tuple(1, 1.1);
 * @endcode
 *
 * If you do not need to reference the tuple later on you can use a unique
 * anonymous name:
 * @code
 * T(int, i, double, d) = std::make_tuple(1, 1.1);
 * @endcode
 *  */

#include <tuple>

#include <type_traits>

#include <util/macro_lib.h>
#include <util/template_lib.h>

// Macros for declaring up to 10 variables and a tuple of references to them
// with a given name (without making that tuple assignable).

/** Initialize one variable and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_1(name, t1, v1)      \
    t1 v1;                                              \
    std::tuple<t1&> name(v1);

/** Initialize two variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_2(name, t1, v1, t2, v2)      \
    t1 v1; t2 v2;                                               \
    std::tuple<t1&,t2&> name(v1, v2);

/** Initialize three variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_3(name, t1, v1, t2, v2, t3, v3)      \
    t1 v1; t2 v2; t3 v3;                                                \
    std::tuple<t1&,t2&,t3&> name(v1, v2, v3);

/** Initialize four variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_4(name, t1, v1, t2, v2, t3, v3, t4, v4) \
    t1 v1; t2 v2; t3 v3; t4 v4;                                         \
    std::tuple<t1&,t2&,t3&,t4&> name(v1, v2, v3, v4);

/** Initialize five variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_5(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5;                                  \
    std::tuple<t1&,t2&,t3&,t4&,t5&> name(v1, v2, v3, v4, v5);

/** Initialize six variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_6(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6;                           \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&> name(v1, v2, v3, v4, v5, v6);

/** Initialize seven variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_7(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7;                    \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&> name(v1, v2, v3, v4, v5, v6, v7);

/** Initialize eight variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_8(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8;             \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&> name(v1, v2, v3, v4, v5, v6, v7, v8);

/** Initialize nine variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_9(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8; t9 v9;      \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&,t9&> name(v1, v2, v3, v4, v5, v6, v7, v8, v9);

/** Initialize ten variables and a tuple 'name'. */
#define NAMED_TUPLE_INITIALIZATION_10(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9, t10, v10) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8; t9 v9; t10 v10; \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&,t9&,t10&> name(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10);

/// Declare variables and a tuple of references to them with given name. This
/// macro calls the correct version of the NAMED_TUPLE_INITIALIZATION
/// macros. The tuple can afterward be used for assigning values to the
/// variables.
#define NAMED_RETURN_TUPLE(...) CONCATENATE(NAMED_TUPLE_INITIALIZATION_,VA_ARGS_NUM_DIV_2(__VA_ARGS__))(__VA_ARGS__)

//====================================================================================================================================//
//====================================================================================================================================//

// Macros for declaring up to 10 variables, a tuple of references to them with
// an ANONYMOUS name, and making that tuple assignable.

/** Initialize one variable and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_1(name, t1, v1)  \
    t1 v1;                                              \
    std::tuple<t1&> name(v1); name

/** Initialize two variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_2(name, t1, v1, t2, v2)  \
    t1 v1; t2 v2;                                               \
    std::tuple<t1&,t2&> name(v1, v2); name

/** Initialize three variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_3(name, t1, v1, t2, v2, t3, v3)  \
    t1 v1; t2 v2; t3 v3;                                                \
    std::tuple<t1&,t2&,t3&> name(v1, v2, v3); name

/** Initialize four variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_4(name, t1, v1, t2, v2, t3, v3, t4, v4) \
    t1 v1; t2 v2; t3 v3; t4 v4;                                         \
    std::tuple<t1&,t2&,t3&,t4&> name(v1, v2, v3, v4); name

/** Initialize five variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_5(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5;                                  \
    std::tuple<t1&,t2&,t3&,t4&,t5&> name(v1, v2, v3, v4, v5); name

/** Initialize six variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_6(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6;                           \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&> name(v1, v2, v3, v4, v5, v6); name

/** Initialize seven variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_7(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7;                    \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&> name(v1, v2, v3, v4, v5, v6, v7); name

/** Initialize eight variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_8(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8;             \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&> name(v1, v2, v3, v4, v5, v6, v7, v8); name

/** Initialize nine variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_9(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8; t9 v9;      \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&,t9&> name(v1, v2, v3, v4, v5, v6, v7, v8, v9); name

/** Initialize ten variables and a tuple 'name'. */
#define ANONYMOUS_TUPLE_INITIALIZATION_10(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9, t10, v10) \
    t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8; t9 v9; t10 v10; \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&,t9&,t10&> name(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10); name

/// Declare variables and a tuple of references to them with an anonymous
/// name. The name is 'unique_tuple_identifyer_LINE' where LINE is the line
/// number. Therefore you must not use this macro more than once in the same
/// line. This macro calls the correct version of the
/// ANONYMOUS_TUPLE_INITIALIZATION macros. These macros and with the tuple name
/// without trailing semicolon so that the tuple can directly be assigned.
#define RETURN_TUPLE(...) CONCATENATE(ANONYMOUS_TUPLE_INITIALIZATION_,VA_ARGS_NUM_DIV_2(__VA_ARGS__))(CONCATENATE(unique_tuple_identifyer_,__LINE__),__VA_ARGS__)

//====================================================================================================================================//
//====================================================================================================================================//

// This are template functions for creating a tuple of references from given
// variables so that thei can be assign in one go.

namespace return_tuple {

    /** Constructs a tuple of references to variable v1. */
    template <class T1>
        std::tuple<T1&> t(T1& v1) {
        return std::tuple<T1&>(v1);
    }

    /** Constructs a tuple of references to variables v1, v2. */
    template <class T1, class T2>
        std::tuple<T1&, T2&> t(T1& v1, T2& v2) {
        return std::tuple<T1&, T2&>(v1, v2);
    }

    /** Constructs a tuple of references to variables v1, v2, v3. */
    template <class T1, class T2, class T3>
        std::tuple<T1&, T2&, T3&> t(T1& v1, T2& v2, T3& v3) {
        return std::tuple<T1&, T2&, T3&>(v1, v2, v3);
    }

    /** Constructs a tuple of references to variables v1, v2, v3, v4. */
    template <class T1, class T2, class T3, class T4>
        std::tuple<T1&, T2&, T3&, T4&> t(T1& v1, T2& v2, T3& v3, T4& v4) {
        return std::tuple<T1&, T2&, T3&, T4&>(v1, v2, v3, v4);
    }

    /** Constructs a tuple of references to variables v1, v2, v3, v4, v5. */
    template <class T1, class T2, class T3, class T4, class T5>
        std::tuple<T1&, T2&, T3&, T4&, T5&> t(T1& v1, T2& v2, T3& v3, T4& v4, T5& v5) {
        return std::tuple<T1&, T2&, T3&, T4&, T5&>(v1, v2, v3, v4, v5);
    }

    /** Constructs a tuple of references to variables v1, v2, v3, v4, v5, v6. */
    template <class T1, class T2, class T3, class T4, class T5, class T6>
        std::tuple<T1&, T2&, T3&, T4&, T5&, T6&> t(T1& v1, T2& v2, T3& v3, T4& v4, T5& v5, T6& v6) {
        return std::tuple<T1&, T2&, T3&, T4&, T5&, T6&>(v1, v2, v3, v4, v5, v6);
    }

    /** Constructs a tuple of references to variables v1, v2, v3, v4, v5, v6, v7. */
    template <class T1, class T2, class T3, class T4, class T5, class T6, class T7>
        std::tuple<T1&, T2&, T3&, T4&, T5&, T6&, T7&> t(T1& v1, T2& v2, T3& v3, T4& v4, T5& v5, T6& v6, T7& v7) {
        return std::tuple<T1&, T2&, T3&, T4&, T5&, T6&, T7&>(v1, v2, v3, v4, v5, v6, v7);
    }

    /** Constructs a tuple of references to variables v1, v2, v3, v4, v5, v6, v7, v8. */
    template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8>
        std::tuple<T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&> t(T1& v1, T2& v2, T3& v3, T4& v4, T5& v5, T6& v6, T7& v7, T8& v8) {
        return std::tuple<T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&>(v1, v2, v3, v4, v5, v6, v7, v8);
    }

    /** Constructs a tuple of references to variables v1, v2, v3, v4, v5, v6, v7, v8, v9. */
    template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8, class T9>
        std::tuple<T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&, T9&> t(T1& v1, T2& v2, T3& v3, T4& v4, T5& v5, T6& v6, T7& v7, T8& v8, T9& v9) {
        return std::tuple<T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&, T9&>(v1, v2, v3, v4, v5, v6, v7, v8, v9);
    }

    /** Constructs a tuple of references to variables v1, v2, v3, v4, v5, v6, v7, v8, v9, v10. */
    template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8, class T9, class T10>
        std::tuple<T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&, T9&, T10&> t(T1& v1, T2& v2, T3& v3, T4& v4, T5& v5, T6& v6, T7& v7, T8& v8, T9& v9, T10& v10) {
        return std::tuple<T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&, T9&, T10&>(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10);
    }

}; // end namespace return_tuple

#endif /* TUPLERETURN_H_ */

