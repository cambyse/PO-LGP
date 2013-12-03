#include <gtest/gtest.h>

#include "../util.h"
#include "../AbstractAction.h"
#include "../Maze/MazeAction.h"
#include "../Maze/AugmentedMazeAction.h"

#include "../debug.h"

namespace {

    // class A {
    // public:
    //     A(int ii = 0): i(ii) {}
    //     virtual int get_i() const { return i; }
    // protected:
    //     virtual void set_i(int ii) { i = ii; }
    // private:
    //     int i;
    // };

    // class B: public A {
    // public:
    //     B(int ii = 1) {
    //         set_i(ii);
    //     }
    //     virtual int get_i() const override { return A::get_i(); }
    // protected:
    //     virtual void set_i(int ii) override { A::set_i(ii); }
    // };

    // class C: public B {
    // public:
    //     C(int ii = 2) {
    //         set_i(ii);
    //     }
    // };

    // TEST(PlayGround, Test) {
    //     A a;
    //     B b;
    //     C c;
    //     DEBUG_OUT(0,"A: " << a.get_i());
    //     DEBUG_OUT(0,"B: " << b.get_i());
    //     DEBUG_OUT(0,"C: " << c.get_i());
    // }

}; // end namespace
