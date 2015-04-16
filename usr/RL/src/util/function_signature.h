#ifndef FUNCTION_SIGNATURE_H_
#define FUNCTION_SIGNATURE_H_

#include <type_traits> // std::remove_reference
#include <functional> // std::function
#include <vector>
#include <tuple>
#include "template_lib.h"

/** \brief This namespace contains some tools to infer the signature of
 * arbitrary functors (objects implementing operator() ). */
namespace function_signature {

    //======================//
    //==== remove_class ====//
    //======================//


    /** \brief Get the signature of C::operator() given class C. */
    template<typename F>
        struct remove_class {};

    /** \brief Specialization for normal C. */
    template<typename C, typename R, typename ... A>
        struct remove_class<R(C::*)(A...)> {
        using type = R(A...);
    };

    /** \brief Specialization for const C. */
    template<typename C, typename R, typename ... A>
        struct remove_class<R(C::*)(A...) const> {
        using type = R(A...);
    };

    /** \brief Specialization for volatile C. */
    template<typename C, typename R, typename ... A>
        struct remove_class<R(C::*)(A...) volatile> {
        using type = R(A...);
    };

    /** \brief Specialization for const volatile C. */
    template<typename C, typename R, typename ... A>
        struct remove_class<R(C::*)(A...) const volatile> {
        using type = R(A...);
    };

    //============================//
    //==== get_signature_impl ====//
    //============================//

    /** \brief Use remove_class to implicitely (via internal type) get signature
     * of F::operator(). */
    template<typename F>
        struct get_signature_impl {
        using type = typename remove_class<decltype(&std::remove_reference<F>::type::operator())>::type;
    };

    template<typename R, typename ... A>
        struct get_signature_impl<R(A...)> {
        using type = R(A...);
    };

    template<typename R, typename ... A>
        struct get_signature_impl<R(&)(A...)> {
        using type = R(A...);
    };

    template<typename R, typename ... A>
        struct get_signature_impl<R(*)(A...)> {
        using type = R(A...);
    };

    //==========================================================//
    //==== get_signature, make_function_type, make_function ====//
    //==========================================================//

    /** \brief Get signature of F::operator(). */
    template<typename F>
        using get_signature = typename get_signature_impl<F>::type;

    /** \brief Get std::function type with a signature equivalent to that of
     * F::operator(). */
    template<typename F>
        using make_function_type = std::function<get_signature<F>>;

    /** \brief Return a std::function object with signature got via
     * make_function_type. */
    template<typename F>
        make_function_type<F> make_function(F &&f) {
        return make_function_type<F>(std::forward<F>(f));
    }

    //==============================//
    //==== get_return_type_impl ====//
    //==============================//

    /** \brief Get the return type implicitely (via internal type). */
    template<typename F>
        struct get_return_type_impl {};

    template<typename R, typename ... A>
        struct get_return_type_impl<std::function<R(A...)>> {
        using type = R;
    };

    template<typename R, typename ... A>
        struct get_return_type_impl<R(A...)> {
        using type = R(A...);
    };

    template<typename R, typename ... A>
        struct get_return_type_impl<R(&)(A...)> {
        using type = R(A...);
    };

    template<typename R, typename ... A>
        struct get_return_type_impl<R(*)(A...)> {
        using type = R(A...);
    };

    //=========================//
    //==== get_return_type ====//
    //=========================//

    /** \brief Get return type of F::operator(). */
    template<typename F>
        using get_return_type = typename get_return_type_impl<make_function_type<F>>::type;

    //=========================//
    //==== get_n_args_impl ====//
    //=========================//

    /** \brief Get the return type implicitely (via internal type). */
    template<typename F>
        struct get_n_args_impl {};

    template<typename R, typename ... A>
        struct get_n_args_impl<std::function<R(A...)>> {
        static const int n_args = sizeof...(A);
    };

    template<typename R, typename ... A>
        struct get_n_args_impl<R(A...)> {
        static const int n_args = sizeof...(A);
    };

    template<typename R, typename ... A>
        struct get_n_args_impl<R(&)(A...)> {
        static const int n_args = sizeof...(A);
    };

    template<typename R, typename ... A>
        struct get_n_args_impl<R(*)(A...)> {
        static const int n_args = sizeof...(A);
    };

    //====================//
    //==== get_n_args ====//
    //====================//

    /** \brief Get return number of arguments of F::operator(). */
    template<typename F>
        constexpr int get_n_args() { return get_n_args_impl<make_function_type<F>>::n_args; }

    //=========================================================//
    //==== arg_type_tuple_type_helper, arg_type_tuple_type ====//
    //=========================================================//

    template<class Func>
        struct arg_type_tuple_type_helper {
            using type = typename arg_type_tuple_type_helper<make_function_type<Func>>::type;
        };

    template<class Ret, class ... Args>
        struct arg_type_tuple_type_helper<std::function<Ret(Args...)>> {
            using type = std::tuple<Args...>;
    };

    template<class Func>
        using arg_type_tuple_type = typename arg_type_tuple_type_helper<Func>::type;

    //==========================================================//
    //==== reversed_tuple_type_helper, reversed_tuple_type  ====//
    //==========================================================//

    template<class TupleType>
        struct reversed_tuple_type_helper { };

    template<class T1, class ... MoreT>
        struct reversed_tuple_type_helper<std::tuple<T1,MoreT...>> {
        using type = typename template_lib::append_tuple_type<
            T1,
            typename reversed_tuple_type_helper<std::tuple<MoreT...>>::type
            >::type;
    };

    template<class T1>
        struct reversed_tuple_type_helper<std::tuple<T1>> {
        using type = std::tuple<T1>;
    };

    template<>
        struct reversed_tuple_type_helper<std::tuple<>> {
        using type = std::tuple<>;
    };

    template<class T>
        using reversed_tuple_type = typename reversed_tuple_type_helper<T>::type;

    //======================================================================================//
    //==== reverse_copy_tuple_idx, recursive_reverse_copy_tuple_idx, reverse_copy_tuple ====//
    //======================================================================================//

    template<int Idx, class T1, class T2>
        void reverse_copy_tuple_idx(const T1& t1, T2& t2) {
        static_assert(std::is_same<reversed_tuple_type<T1>,T2>(),"Second type is not reverse of first type");
        static_assert(Idx<=std::tuple_size<T1>::value,"Index too large for given tuple type");
        static_assert(Idx>0,"Index too small");
        std::get<std::tuple_size<T1>::value-Idx>(t2) = std::get<Idx-1>(t1);
    }

    template<int Idx, class T1, class T2>
        struct recursive_reverse_copy_tuple_idx {
            static void do_it(const T1& t1, T2& t2) {
                reverse_copy_tuple_idx<Idx,T1,T2>(t1,t2);
                recursive_reverse_copy_tuple_idx<Idx-1,T1,T2>::do_it(t1,t2);
            }
        };

    template<class T1, class T2>
        struct recursive_reverse_copy_tuple_idx<0,T1,T2> {
            static void do_it(const T1&, T2&) { }
        };

    template<class T1, class T2>
        void reverse_copy_tuple(const T1& t1, T2& t2) {
        static_assert(std::is_same<reversed_tuple_type<T1>,T2>(),"Second type is not reverse of first type");
        recursive_reverse_copy_tuple_idx<std::tuple_size<T1>::value,T1,T2>::do_it(t1,t2);
    }

    //=======================//
    //==== reverse_tuple ====//
    //=======================//

    template<class T1>
        reversed_tuple_type<T1> reverse_tuple(const T1& t1) {
        typedef reversed_tuple_type<T1> t2_t;
        t2_t t2;
        reverse_copy_tuple<T1,t2_t>(t1,t2);
        return t2;
    }

    //=====================================================//
    //==== tuple_get_first_type, tuple_drop_first_type ====//
    //=====================================================//

    template<class T>
        struct tuple_get_first_type { };

    template<class T1, class ... TRest>
        struct tuple_get_first_type<std::tuple<T1,TRest...>> {
        using type = T1;
    };

    template<class T>
        struct tuple_drop_first_type { };

    template<class T1, class ... TRest>
        struct tuple_drop_first_type<std::tuple<T1,TRest...>> {
        using type = std::tuple<TRest...>;
    };

    //===========================//
    //==== zipped_tuple_type ====//
    //===========================//

    // general version using recursion
    template<class T1, class T2>
        struct zipped_tuple_type {
            static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples must have the same size to zip them");
            using type = typename template_lib::prepend_tuple_type<
                std::tuple<typename tuple_get_first_type<T1>::type, typename tuple_get_first_type<T2>::type>,
                typename zipped_tuple_type<typename tuple_drop_first_type<T1>::type, typename tuple_drop_first_type<T2>::type>::type
                >::type;
        };

    // specialization for tuples with one element
    template<class T1, class T2>
        struct zipped_tuple_type<std::tuple<T1>,std::tuple<T2>> {
        using type = std::tuple<std::tuple<T1, T2>>;
    };

    // specialization for empty tuples
    template<>
        struct zipped_tuple_type<std::tuple<>,std::tuple<>> {
        using type = std::tuple<>;
        };

    //=============================================================================//
    //==== copy_zip_tuples_idx, recursive_copy_zip_tuples_idx, copy_zip_tuples ====//
    //=============================================================================//

    template<int Idx, class T1, class T2, class T3>
        void copy_zip_tuples_idx(const T1& t1, const T2& t2, T3& t3) {
        static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples of unequal size cannot be zipped");
        static_assert(std::is_same<typename zipped_tuple_type<T1,T2>::type,T3>(),"Third type is not zipped type of first two types");
        static_assert(Idx<=std::tuple_size<T3>::value,"Index too large for given tuple types");
        std::get<Idx-1>(t3) = std::make_tuple(std::get<Idx-1>(t1),std::get<Idx-1>(t2));
    }

    template<int Idx, class T1, class T2, class T3>
        struct recursive_copy_zip_tuples_idx {
            static void do_it(const T1& t1, const T2& t2, T3& t3) {
                copy_zip_tuples_idx<Idx,T1,T2,T3>(t1,t2,t3);
                recursive_copy_zip_tuples_idx<Idx-1,T1,T2,T3>::do_it(t1,t2,t3);
            }
        };

    template<class T1, class T2, class T3>
        struct recursive_copy_zip_tuples_idx<0,T1,T2,T3> {
        static void do_it(const T1&, const T2&, T3&) { }
    };

    template<class T1, class T2, class T3>
        void copy_zip_tuples(const T1& t1, const T2& t2, T3& t3) {
        static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples of unequal size cannot be zipped");
        static_assert(std::is_same<typename zipped_tuple_type<T1,T2>::type,T3>(),"Third type is not zipped type of first two types");
        recursive_copy_zip_tuples_idx<std::tuple_size<T1>::value,T1,T2,T3>::do_it(t1,t2,t3);
    }

    //====================//
    //==== zip_tuples ====//
    //====================//

    template<class T1, class T2>
        typename zipped_tuple_type<T1,T2>::type zip_tuples(const T1& t1, const T2& t2) {
        typedef typename zipped_tuple_type<T1,T2>::type T3;
        T3 t3;
        copy_zip_tuples<T1,T2,T3>(t1,t2,t3);
        return t3;
    }

    //===============================================================//
    //==== unzipped_tuple_type_first, unzipped_tuple_type_second ====//
    //===============================================================//

    template<class T1>
        struct unzipped_tuple_type_first { };

    template<class First, class ... Rest>
        struct unzipped_tuple_type_first<std::tuple<First,Rest...>> {
            static_assert(std::tuple_size<First>::value==2,"Can only unzip tuples of pairs (size of first element is not two)");
            using type = typename template_lib::prepend_tuple_type<
                typename std::tuple_element<0,First>::type,
                typename unzipped_tuple_type_first<std::tuple<Rest...>>::type
                >::type;
    };

    template<class First>
        struct unzipped_tuple_type_first<std::tuple<First>> {
        static_assert(std::tuple_size<First>::value==2,"Can only unzip tuples of pairs (size of first element is not two)");
        using type = std::tuple<typename std::tuple_element<0,First>::type>;
    };

    template<>
        struct unzipped_tuple_type_first<std::tuple<>> {
        using type = std::tuple<>;
    };

    template<class T1>
        struct unzipped_tuple_type_second { };

    template<class First, class ... Rest>
        struct unzipped_tuple_type_second<std::tuple<First,Rest...>> {
            static_assert(std::tuple_size<First>::value==2,"Can only unzip tuples of pairs (size of first element is not two)");
            using type = typename template_lib::prepend_tuple_type<
                typename std::tuple_element<1,First>::type,
                typename unzipped_tuple_type_second<std::tuple<Rest...>>::type
                >::type;
    };

    template<class First>
        struct unzipped_tuple_type_second<std::tuple<First>> {
        static_assert(std::tuple_size<First>::value==2,"Can only unzip tuples of pairs (size of first element is not two)");
        using type = std::tuple<typename std::tuple_element<1,First>::type>;
    };

    template<>
        struct unzipped_tuple_type_second<std::tuple<>> {
        using type = std::tuple<>;
    };

    //========================================================================================================//
    //==== copy_unzip_tuples_first_idx, recursive_copy_unzip_tuples_first_idx, copy_unzip_tuples_first    ====//
    //==== copy_unzip_tuples_second_idx, recursive_copy_unzip_tuples_second_idx, copy_unzip_tuples_second ====//
    //========================================================================================================//

    template<int Idx, class T1, class T2>
        void copy_unzip_tuples_first_idx(const T1& t1, T2& t2) {
        static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples have unequal size");
        static_assert(std::is_same<typename unzipped_tuple_type_first<T1>::type,T2>(),"Types do not match");
        static_assert(Idx<=std::tuple_size<T1>::value,"Index too large for given tuple types");
        static_assert(Idx>0,"Index too small");
        std::get<Idx-1>(t2) = std::get<0>(std::get<Idx-1>(t1));
    }

    template<int Idx, class T1, class T2>
        struct recursive_copy_unzip_tuples_first_idx {
            static void do_it(const T1& t1, T2& t2) {
                copy_unzip_tuples_first_idx<Idx,T1,T2>(t1,t2);
                recursive_copy_unzip_tuples_first_idx<Idx-1,T1,T2>::do_it(t1,t2);
            }
        };

    template<class T1, class T2>
        struct recursive_copy_unzip_tuples_first_idx<0,T1,T2> {
        static void do_it(const T1&, T2&) { }
    };

    template<class T1, class T2>
        void copy_unzip_tuples_first(const T1& t1, T2& t2) {
        static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples have unequal size");
        static_assert(std::is_same<typename unzipped_tuple_type_first<T1>::type,T2>(),"Types do not match");
        recursive_copy_unzip_tuples_first_idx<std::tuple_size<T1>::value,T1,T2>::do_it(t1,t2);
    }

    template<int Idx, class T1, class T2>
        void copy_unzip_tuples_second_idx(const T1& t1, T2& t2) {
        static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples have unequal size");
        static_assert(std::is_same<typename unzipped_tuple_type_second<T1>::type,T2>(),"Types do not match");
        static_assert(Idx<=std::tuple_size<T1>::value,"Index too large for given tuple types");
        static_assert(Idx>0,"Index too small");
        std::get<Idx-1>(t2) = std::get<1>(std::get<Idx-1>(t1));
    }

    template<int Idx, class T1, class T2>
        struct recursive_copy_unzip_tuples_second_idx {
            static void do_it(const T1& t1, T2& t2) {
                copy_unzip_tuples_second_idx<Idx,T1,T2>(t1,t2);
                recursive_copy_unzip_tuples_second_idx<Idx-1,T1,T2>::do_it(t1,t2);
            }
        };

    template<class T1, class T2>
        struct recursive_copy_unzip_tuples_second_idx<0,T1,T2> {
        static void do_it(const T1&, T2&) { }
    };

    template<class T1, class T2>
        void copy_unzip_tuples_second(const T1& t1, T2& t2) {
        static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples have unequal size");
        static_assert(std::is_same<typename unzipped_tuple_type_second<T1>::type,T2>(),"Types do not match");
        recursive_copy_unzip_tuples_second_idx<std::tuple_size<T1>::value,T1,T2>::do_it(t1,t2);
    }

    //=================================================//
    //==== unzip_tuples_first, unzip_tuples_second ====//
    //=================================================//

    template<class T1>
        typename unzipped_tuple_type_first<T1>::type unzip_tuples_first(const T1& t1) {
        typedef typename unzipped_tuple_type_first<T1>::type T2;
        T2 t2;
        copy_unzip_tuples_first<T1,T2>(t1,t2);
        return t2;
    }

    template<class T1>
        typename unzipped_tuple_type_second<T1>::type unzip_tuples_second(const T1& t1) {
        typedef typename unzipped_tuple_type_second<T1>::type T2;
        T2 t2;
        copy_unzip_tuples_second<T1,T2>(t1,t2);
        return t2;
    }

    //==================================================//
    //==== map_tuples_idx, recursive_map_tuples_idx ====//
    //==================================================//

    template<class T1, class T2, template<class,class> class Mapper, int Idx>
        struct map_tuples_idx {
            static_assert(Idx>0,"Index too small");
            static_assert(Idx<=std::tuple_size<T1>::value,"Index is too large for first tuple");
            static_assert(Idx<=std::tuple_size<T2>::value,"Index is too large for second tuple");
            typedef typename std::tuple_element<Idx-1,T1>::type T1_elem;
            typedef typename std::tuple_element<Idx-1,T2>::type T2_elem;
            typedef Mapper<T1_elem,T2_elem> mapper_t;
            static_assert(mapper_t::can_do,"Mapper type cannot map the requested types");
            static void get(const T1& t1, T2& t2) {
                std::get<Idx-1>(t2) = mapper_t::map(std::get<Idx-1>(t1));
            }
        };

    template<class T1, class T2, template<class,class> class Mapper, int Idx>
        struct recursive_map_tuples_idx {
            static void get (const T1& t1, T2& t2) {
                map_tuples_idx<T1,T2,Mapper,Idx>::get(t1,t2);
                recursive_map_tuples_idx<T1,T2,Mapper,Idx-1>::get(t1,t2);
            }
        };

    template<class T1, class T2, template<class,class> class Mapper>
        struct recursive_map_tuples_idx<T1,T2,Mapper,0> {
            static void get (const T1&, T2&) { }
        };

    //====================//
    //==== map_tuples ====//
    //====================//

    template<class T1, class T2, template<class,class> class Mapper>
        void map_tuples(const T1& t1, T2& t2) {
        static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples do not have equal size");
        return recursive_map_tuples_idx<T1,T2,Mapper,std::tuple_size<T1>::value>::get(t1,t2);
    }

    //=====================================//
    //==== all_elements_below_idx_true ====//
    //=====================================//

    template<int Idx, class TupleType>
        struct all_elements_below_idx_true {
            static_assert(Idx<=std::tuple_size<TupleType>::value,"Index too large for tuple");
            static bool get(TupleType t) {
                return std::get<Idx-1>(t) && all_elements_below_idx_true<Idx-1,TupleType>::get(t);
            }
        };

    template<class TupleType>
        struct all_elements_below_idx_true<0,TupleType> {
            static bool get(TupleType) {
                return true;
            }
        };

    //===========================//
    //==== all_elements_true ====//
    //===========================//

    template<class TupleType>
        bool all_elements_true(TupleType t) {
        return all_elements_below_idx_true<std::tuple_size<TupleType>::value,TupleType>::get(t);
    }

    //=========================//
    //==== bind_first_arg  ====//
    //=========================//

    /** \brief Returns a function with first argument bound. */
    template<class Ret, class FirstArg, class SecondArg, class ... RestArgs>
        std::function<Ret(SecondArg,RestArgs...)> bind_first_arg(std::function<Ret(FirstArg,SecondArg,RestArgs...)> f, FirstArg first_arg) {
        return [=](SecondArg second_arg, RestArgs ... rest_args) { return f(first_arg,second_arg,rest_args...); };
    }

    /** \brief Overload for functions with only one argument. */
    template<class Ret, class FirstArg>
        std::function<Ret()> bind_first_arg(std::function<Ret(FirstArg)> f, FirstArg first_arg) {
        return [=]() { return f(first_arg); };
    }

    //=======================================//
    //==== bind_first_arg_with_tuple_idx ====//
    //=======================================//

    /** \brief Returns a function with first argument bound using element from tuple given by Idx-1. */
    template<class Func, class TupleType, int Idx>
        auto bind_first_arg_with_tuple_idx(Func f, TupleType t) -> decltype(bind_first_arg(f,std::get<Idx-1>(t))) {
        return bind_first_arg(f, std::get<Idx-1>(t));
    }

    //====================================================//
    //==== first_arg_recursively_bound_with_tuple_idx ====//
    //====================================================//

    /** \brief Binds all arguments of a function (starting with first argument)
     * with elements from tuple (starting at Idx, counting down).
     *
     * Note that the order of function aruments and tuple elements is inverse to
     * each other.*/
    template<class Func, class TupleType, int Idx>
        struct first_arg_recursively_bound_with_tuple_idx {
            static_assert(Idx<=std::tuple_size<TupleType>::value,"Index too large for tuple");
            static_assert(Idx>=get_n_args<Func>(),"Cannot bind all function arguments (index would become too small)");
            static std::function<get_return_type<Func>()> get(Func f, TupleType t) {
                auto ff = bind_first_arg_with_tuple_idx<Func,TupleType,Idx>(f,t);
                return first_arg_recursively_bound_with_tuple_idx<decltype(ff),TupleType,Idx-1>::get(ff,t);
            }
        };

    /** \brief Specialization for Idx=0. */
    template<class Func, class TupleType>
        struct first_arg_recursively_bound_with_tuple_idx<Func,TupleType,0> {
        static_assert(get_n_args<Func>()<=1,"Cannot bind all function arguments (more than one left but index is already zero)");
            static std::function<get_return_type<Func>()> get(Func f, TupleType) {
                return f;
            }
        };

    //============================//
    //==== bind_tuple_as_args ====//
    //============================//

    /** \brief Binds all arguments of a function with tuple elements. */
    template<class Func, class TupleType>
        std::function<get_return_type<Func>()> bind_tuple_as_args(Func f, TupleType t) {
        auto t_reverse = reverse_tuple(t);
        return first_arg_recursively_bound_with_tuple_idx<Func,decltype(t_reverse),get_n_args<Func>()>::get(f,t_reverse);
    }

    //============================//
    //==== array_to_tuple_idx ====//
    //============================//

    template<class ArrType, class TupleType, int Idx>
        struct array_to_tuple_idx {
            static void get(const ArrType& a, TupleType& t) {
                //static_assert(a.size()==std::tuple_size<TupleType>::value,"Array and tuple do not have equal size");
                static_assert(Idx<=std::tuple_size<TupleType>::value,"Index too large");
                static_assert(Idx>0,"Index negative");
                std::get<Idx-1>(t) = a[Idx-1];
                array_to_tuple_idx<ArrType,TupleType,Idx-1>::get(a,t);
            }
        };

    template<class ArrType, class TupleType>
        struct array_to_tuple_idx<ArrType,TupleType,0> {
            static void get(const ArrType&, TupleType&) { }
        };

    //========================//
    //==== array_to_tuple ====//
    //========================//

    template<class ArrType, class TupleType>
        void array_to_tuple(const ArrType& a, TupleType& t) {
        array_to_tuple_idx<ArrType,TupleType,std::tuple_size<TupleType>::value>::get(a,t);
    }

    template<class ArrType, class TupleType>
        TupleType array_to_tuple(const ArrType& a) {
        TupleType t;
        array_to_tuple_idx<ArrType,TupleType,std::tuple_size<TupleType>::value>::get(a,t);
        return t;
    }

    //============================//
    //==== tuple_to_array_idx ====//
    //============================//

    template<class TupleType, class ArrayType, int Idx>
        struct tuple_to_array_idx {
            static void get(const TupleType& t, ArrayType& a) {
                //static_assert(a.size()==std::array_size<ArrayType>::value,"tuple and array do not have equal size");
                a[Idx-1] = std::get<Idx-1>(t);
                tuple_to_array_idx<TupleType,ArrayType,Idx-1>::get(t,a);
            }
        };

    template<class TupleType, class ArrayType>
        struct tuple_to_array_idx<TupleType,ArrayType,0> {
            static void get(const TupleType&, ArrayType&) { }
        };

    //========================//
    //==== tuple_to_array ====//
    //========================//

    template<class TupleType, class ArrayType>
        void tuple_to_array(const TupleType& t, ArrayType& a) {
        tuple_to_array_idx<TupleType,ArrayType,std::tuple_size<TupleType>::value>::get(t,a);
    }

    template<class TupleType, class ArrayType>
        ArrayType tuple_to_array(const TupleType& t) {
        ArrayType a;
        tuple_to_array_idx<TupleType,ArrayType,std::tuple_size<TupleType>::value>::get(t,a);
        return a;
    }
};

#endif /* FUNCTION_SIGNATURE_H_ */
