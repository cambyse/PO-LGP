#ifndef TEMPLATE_LIB_H_
#define TEMPLATE_LIB_H_

#include <tuple>
#include <array>

namespace template_lib {

    //============================//
    // Prepended types to a tuple //
    //============================//

    /// Prepend N times type T as a recursion.
    template<size_t N, class T, class ... TupleTypes>
        struct prepend_N_tuple_type {
            using type = typename prepend_N_tuple_type<N-1,T,T,TupleTypes...>::type;
    };

    /// Specialization (N=0) to terminate recursion.
    template<class T, class ... TupleTypes>
        struct prepend_N_tuple_type<0,T,TupleTypes...> {
        using type = std::tuple<TupleTypes...>;
    };

    /// For non-tuple class don't define a type.
    template<class T, class NonTuple>
        struct prepend_tuple_type {};

    /// Specialization for tuples. Prepends T to TupleTypes.
    template<class T, class ... TupleTypes>
        struct prepend_tuple_type<T,std::tuple<TupleTypes...>> {
        using type = typename prepend_N_tuple_type<1,T,TupleTypes...>::type;
    };

    //===========================//
    // Appended types to a tuple //
    //===========================//

    /// Append N times type T as a recursion.
    template<int N, class T, class ... TupleTypes>
        struct append_N_tuple_type {
            using type = typename append_N_tuple_type<N-1,T,TupleTypes...,T>::type;
    };

    /// Specialization (N=0) to terminate recursion.
    template<class T, class ... TupleTypes>
        struct append_N_tuple_type<0,T,TupleTypes...> {
        using type = std::tuple<TupleTypes...>;
    };

    /// For non-tuple class don't define a type.
    template<class T, class Tuple>
        struct append_tuple_type {};

    /// Specialization for tuples. Appends T to TupleTypes.
    template<class T, class ... TupleTypes>
        struct append_tuple_type<T,std::tuple<TupleTypes...>> {
        using type = typename append_N_tuple_type<1,T,TupleTypes...>::type;
    };

    //================================//
    // A tuple with N identical types //
    //================================//

    /** Tuple of N times type T. For instance @code N_tuple<3,int> @endcode is
     * derived from @code std::tuple<int,int,int> @endcode. */
    /* template<int N, class T> */
    /*     struct N_tuple: public template_lib::prepend_N_tuple_type<N,T>::type { */
    /*     // typedef */
    /*     typedef typename template_lib::prepend_N_tuple_type<N,T>::type type; */
    /*     N_tuple() {} */
    /*     /// Constructor for type casting array to tuple. */
    /*     N_tuple(const std::array<T,N> & arr); // implemented below */
    /* }; */

    template<int N, class T>
        using N_tuple = typename prepend_N_tuple_type<N,T>::type;

    //================================================//
    // Copy values between tuples of different length //
    //================================================//

    template <size_t N, class ... T>
        struct tuple_copy {};

    template <size_t N, class ... TupleTypes_1, class ... TupleTypes_2>
        struct tuple_copy<N,std::tuple<TupleTypes_1...>,std::tuple<TupleTypes_2...>> {
        typedef std::tuple<TupleTypes_1...> T1;
        typedef std::tuple<TupleTypes_2...> T2;
        static void copy(const T1 & t1, T2 & t2) {
            std::get<N>(t2) = std::get<N>(t1);
            tuple_copy<N-1,T1,T2>::copy(t1,t2);
        }
    };

    template <class ... TupleTypes_1, class ... TupleTypes_2>
        struct tuple_copy<0,std::tuple<TupleTypes_1...>,std::tuple<TupleTypes_2...>> {
        typedef std::tuple<TupleTypes_1...> T1;
        typedef std::tuple<TupleTypes_2...> T2;
        static void copy(const T1 & t1, T2 & t2) {
            std::get<0>(t2) = std::get<0>(t1);
        }
    };

    //=================================//
    // Copy values from array to tuple //
    //=================================//

    /// Class with a single static member function copy_values::copy. Iterating
    /// through the entries is done via recursion.
    template <size_t IDX, size_t N, class TYPE>
        struct copy_values {
            /// Copies values an array to a tuple.
            static void copy(N_tuple<N,TYPE> & tuple,
                             const std::array<TYPE,N> & array) {
                std::get<IDX>(tuple) = array[IDX];
                copy_values<IDX-1, N, TYPE>::copy(tuple, array);
            }
        };

    /// Template specialization to terminate recursion.
    template <size_t N, class TYPE>
        struct copy_values<0,N,TYPE> {
        static void copy(N_tuple<N,TYPE> & tuple,
                         const std::array<TYPE,N> & array) {
            std::get<0>(tuple) = array[0];
        }
    };

    /// This function converts an array to a tuple. Values are copied using
    /// copy_values::copy.
    template <size_t N, class TYPE>
        N_tuple<N,TYPE> array_to_tuple(const std::array<TYPE,N> & arr) {
        N_tuple<N,TYPE> return_tuple;
        copy_values<N-1,N,TYPE>::copy(return_tuple, arr);
        return return_tuple;
    }

    // now implement constructor from array using converter functions from above
    /* template<int N, class T> */
    /* N_tuple<N,T>::N_tuple(const std::array<T,N> & arr): type(array_to_tuple(arr)) {} */

} // namespace template_lib

#endif /* TEMPLATE_LIB_H_ */
