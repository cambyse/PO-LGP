#include <gtest/gtest.h>

#include "../util/Commander.h"

#define DEBUG_STRING ""
#include "../util/debug.h"


//------------------------------------------------------------

template<class ... T>
class A {
public:
    void print() {
        DEBUG_OUT(0,"A: " << sizeof...(T));
    }
};

template<class ... FTYPE,template<class ...> class T, class ... TARG>
void f(T<TARG...> t) {
    //static_assert(std::is_same<FTYPE..., TARG...>::value,"Not the same type.");
    DEBUG_OUT(0,sizeof...(FTYPE));
    t.print();
}

//------------------------------------------------------------

template<class ... A1REST,
         class ... A2REST>
constexpr bool all_same(A<A1REST...>, A<A2REST...>) {
    return sizeof...(A1REST)==0 && sizeof...(A2REST)==0;
}

template<class A1First,
         class A2First>
constexpr bool all_same(A<A1First>, A<A2First>) {
    return std::is_same<A1First,A2First>::value;
}

template<class A1First, class ... A1REST,
         class A2First, class ... A2REST>
constexpr bool all_same(A<A1First, A1REST...>, A<A2First, A2REST...>) {
    return sizeof...(A1REST)==sizeof...(A2REST) &&
        std::is_same<A1First,A2First>::value && all_same(A<A1REST...>(),A<A2REST...>());
}

// static_assert(all_same(A<int,double,int>(),A<int,double,int>()), "error 1"); // OK
// static_assert(all_same(A<int,double,int>(),A<int,int>()), "error 2");        // different number
// static_assert(all_same(A<int,double,int>(),A<int,bool,int>()), "error 3");   // different type

//------------------------------------------------------------

template<template<class...> class T>
void f(T t) {
    std::function<void(int)> ff(t);
    ff(3);
}

TEST(Commander, TMP) {

    f([](int){});

    using namespace Commander;

    //----------------------------------//
    // Want to be able to do this

    // int ext = 5;
    // CommandCenter c;
    // c.add_command(
    //     {"first-alias","second-alias"},
    //     [=](int i, double d = 3.3) -> ReturnType {
    //         if(i<0) {
    //             return {false, "First argument must be a positive integer"};
    //         } else {
    //             DEBUG_OUT(0,"This is a lambda closure using extern variable ext=" << ext <<
    //                       " plus two arguments i=" << i << " and d=" << d);
    //             return {true, "I successfully printed a message"};
    //         }
    //     },
    //     "This is a description of the command"
    //     );
    // DEBUG_OUT(0,c.get_help());
    // DEBUG_OUT(0,c.execute("first-alias -2 3.4")); // error message
    // DEBUG_OUT(0,c.execute("second-alias 5")); // success

    //----------------------------------//

    //static_assert(std::is_same<bool,int>::value,"test failed");

    //f<int,double>(A<int,double>());

    //g(A<int,int,int>(),A<int>());

    return;

    // int i = 5;

    // CommandCenter c;
    // c.add(CommandList("do_nothing"),
    //       Arguments<>(),
    //       Arguments<>(),
    //       Description("Prints \"I don't do anything!\""),
    //       [](){DEBUG_OUT(0,"I don't do anything!");}
    //     );
    // c.add(CommandList("print_i"),
    //       Arguments<>(),
    //       Arguments<>(),
    //       Description("Prints variable i from CommanderTest body."),
    //       [&](){DEBUG_OUT(0,"i = " << i);}
    //     );

    // auto com = c.print_commands();
    // DEBUG_OUT(0,"");
    // for(auto line : com) {
    //     DEBUG_OUT(0,line);
    // }
    // DEBUG_OUT(0,"");

    // c.execute("do_nothing");
    // c.execute("print_i");

    // auto func1 = [] () { DEBUG_OUT(0,"empty lambda"); };
    // auto func2 = [=] () { DEBUG_OUT(0,"print i: " << i); };
    // func2();
    // func1();
}
