#ifndef FUNCTION_SIGNATURE_H_
#define FUNCTION_SIGNATURE_H_

#include <type_traits> // std::remove_reference
#include <typeindex> // std::type_index
#include <functional> // std::function
#include <vector>

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

    /** \brief Get return type of F::operator(). */
    template<typename F>
        constexpr int get_n_args() { return get_n_args_impl<make_function_type<F>>::n_args; }

    //==============================//
    //==== get_arg_type_indices ====//
    //==============================//

    /** \brief Get vector of argument types (via type_index). */
    /* template<typename ... Args> */
    /*     constexpr std::vector<std::type_index> get_arg_type_indices() { */
    /*     static_assert(sizeof...(Args)==0, */
    /*                   "This function should only be used for empty template lists" */
    /*         ); */
    /*     return {}; */
    /* } */

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

    //========================//
    //==== recursive_bind ====//
    //========================//

    /** \brief Recursively bind function parameters. */
    template<class ParamType, int ParamIdx, template<class,class> class Interpreter, class Ret, class FirstArg>
        std::function<Ret()> recursive_bind(const ParamType arr[], std::function<Ret(FirstArg)> f) {
        const Interpreter<ParamType,FirstArg> interp;
        return std::bind(f, interp(arr[ParamIdx]));
    }

    /** \brief Recursively bind function parameters. */
    template<class ParamType, int ParamIdx, template<class,class> class Interpreter, class Ret, class FirstArg, class SecondArg, class ... Rest>
        std::function<Ret()> recursive_bind(const ParamType arr[], std::function<Ret(FirstArg,SecondArg,Rest...)> f) {
        const Interpreter<ParamType,FirstArg> interp;
        auto ff = std::bind(f, std::placeholders::_1, interp(arr[ParamIdx]));
        return recursive_bind<ParamType,ParamIdx+1,Interpreter,Ret,SecondArg,Rest...>(arr,ff);
    }
};

#endif /* FUNCTION_SIGNATURE_H_ */
