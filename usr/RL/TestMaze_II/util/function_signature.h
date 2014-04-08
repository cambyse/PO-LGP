#ifndef FUNCTION_SIGNATURE_H_
#define FUNCTION_SIGNATURE_H_

#include <type_traits> // std::remove_reference
#include <typeindex> // std::type_index
#include <functional> // std::function
#include <vector>
#include <tuple>

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

    //==============================//
    //==== get_arg_type_indices ====//
    //==============================//

    /** \brief Get vector of argument types (via type_index). */
    template<typename FirstArg>
        constexpr std::vector<std::type_index> get_arg_type_indices() {
        return {std::type_index(typeid(FirstArg))};
    }

    /** \brief Get vector of argument types (via type_index). */
    template<typename FirstArg, typename SecondArg, typename ... RestArgs>
        constexpr std::vector<std::type_index> get_arg_type_indices() {
        auto v1 = get_arg_type_indices<FirstArg>();
        auto v2 = get_arg_type_indices<SecondArg,RestArgs...>();
        v1.insert(v1.end(),v2.begin(),v2.end());
        return v1;
    }

    /** \brief Get vector of argument types (via type_index) of a
     * std::function. */
    template<typename Ret, typename ... Args>
        constexpr std::vector<std::type_index> get_arg_type_indices(std::function<Ret(Args...)>) {
        return get_arg_type_indices<Args...>();
    }

    /** \brief Get vector of argument types (via type_index) of an arbitrary
     * functor. */
    template<typename F>
        constexpr std::vector<std::type_index> get_arg_type_indices(F f) {
        return get_arg_type_indices(make_function(f));
    }

    //===============================//
    //==== get_return_type_index ====//
    //===============================//

    /** \brief Get the return type (via type_index) of a std::function. */
    template<typename Ret, typename ... Args>
        constexpr std::type_index get_return_type_index(std::function<Ret(Args...)>) {
        return std::type_index(typeid(Ret));
    }

    /** \brief Get the return type (via type_index) of an arbitrary functor. */
    template<typename F>
        constexpr std::type_index get_return_type_index(F f) {
        return get_return_type_index(make_function(f));
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

    /** \brief Returns a function with first argument bound using element from tuple given by Idx. */
    template<class Func, class TupleType, int Idx>
        auto bind_first_arg_with_tuple_idx(Func f, TupleType t) -> decltype(bind_first_arg(f,std::get<Idx>(t))) {
        return bind_first_arg(f, std::get<Idx>(t));
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
            static_assert(std::tuple_size<TupleType>::value>Idx,"Tuple has not enough elements");
            static_assert(get_n_args<Func>()<=Idx+1,"Cannot bind all function arguments (index for tuple element would become negative before all arguments are bound)");
            static std::function<get_return_type<Func>()> get(Func f, TupleType t) {
                auto ff = bind_first_arg_with_tuple_idx<Func,TupleType,Idx>(f,t);
                return first_arg_recursively_bound_with_tuple_idx<decltype(ff),TupleType,Idx-1>::get(ff,t);
            }
        };

    /** \brief Specialization for Idx=0. */
    template<class Func, class TupleType>
        struct first_arg_recursively_bound_with_tuple_idx<Func,TupleType,0> {
        static_assert(get_n_args<Func>()<=1,"Cannot bind all function arguments (more than one left but index is already zero)");
            static std::function<get_return_type<Func>()> get(Func f, TupleType t) {
                return bind_first_arg_with_tuple_idx<Func,TupleType,0>(f,t);
            }
        };

    //============================//
    //==== bind_tuple_as_args ====//
    //============================//

    /** \brief Binds all arguments of a function with tuple elements (last
     * element of tuple will be bound to first argument of function and vice
     * versa). */
    template<class Func, class TupleType>
        std::function<get_return_type<Func>()> bind_tuple_as_args(Func f, TupleType t) {
        return first_arg_recursively_bound_with_tuple_idx<Func,TupleType,get_n_args<Func>()-1>::get(f,t);
    }

    //=====================================//
    //==== all_elements_below_idx_true ====//
    //=====================================//

    template<int Idx, class TupleType>
        struct all_elements_below_idx_true {
            static_assert(std::tuple_size<TupleType>::value>=Idx,"Index is larger than tuple size");
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

    //==================================================//
    //==== map_tuples_idx, recursive_map_tuples_idx ====//
    //==================================================//

    template<class T1, class T2, template<class,class> class Mapper, int Idx>
        struct map_tuples_idx {
            static_assert(Idx>=0,"Cannot use negative index to access tuple element");
            static_assert(Idx<std::tuple_size<T1>::value,"Index is too large for first tuple");
            static_assert(Idx<std::tuple_size<T2>::value,"Index is too large for second tuple");
            typedef typename std::tuple_element<Idx,T1>::type T1_elem;
            typedef typename std::tuple_element<Idx,T2>::type T2_elem;
            typedef Mapper<T1_elem,T2_elem> mapper_t;
            static_assert(mapper_t::can_do,"Mapper type cannot map the requested types");
            static void get(const T1& t1, T2& t2) {
                std::get<Idx>(t2) = mapper_t::map(std::get<Idx>(t1));
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
            static void get (const T1& t1, T2& t2) {
                map_tuples_idx<T1,T2,Mapper,0>::get(t1,t2);
            }
        };

    //====================//
    //==== map_tuples ====//
    //====================//

    template<class T1, class T2, template<class,class> class Mapper>
        void map_tuples(const T1& t1, T2& t2) {
        static_assert(std::tuple_size<T1>::value==std::tuple_size<T2>::value,"Tuples do not have equal size");
        return recursive_map_tuples_idx<T1,T2,Mapper,std::tuple_size<T1>::value-1>::get(t1,t2);
    }

    //============================//
    //==== array_to_tuple_idx ====//
    //============================//

    template<class ArrType, class TupleType, int Idx>
        struct array_to_tuple_idx {
            static void get(const ArrType& a, TupleType& t) {
                //static_assert(a.size()==std::tuple_size<TupleType>::value,"Array and tuple do not have equal size");
                std::get<Idx>(t) = a[Idx];
                array_to_tuple_idx<ArrType,TupleType,Idx-1>::get(a,t);
            }
        };

    template<class ArrType, class TupleType>
        struct array_to_tuple_idx<ArrType,TupleType,0> {
            static void get(const ArrType& a, TupleType& t) {
                //static_assert(a.size()==std::tuple_size<TupleType>::value,"Array and tuple do not have equal size");
                std::get<0>(t) = a[0];
            }
        };

    //========================//
    //==== array_to_tuple ====//
    //========================//

    template<class ArrType, class TupleType>
        void array_to_tuple(const ArrType& a, TupleType& t) {
        array_to_tuple_idx<ArrType,TupleType,std::tuple_size<TupleType>::value-1>::get(a,t);
    }

    template<class ArrType, class TupleType>
        TupleType array_to_tuple(const ArrType& a) {
        TupleType t;
        array_to_tuple_idx<ArrType,TupleType,std::tuple_size<TupleType>::value-1>::get(a,t);
        return t;
    }

};

#endif /* FUNCTION_SIGNATURE_H_ */
