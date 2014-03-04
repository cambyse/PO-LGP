#include <gtest/gtest.h>

#include <memory> // for shared_ptr

#include "../util/debug.h"

using std::shared_ptr;

TEST(SandBox, SharedPtr) {
    shared_ptr<int> i1(new int(0));
    shared_ptr<int> i2(new int(0));
    shared_ptr<int> i3(i1);
    DEBUG_OUT(0,"i1 (" << i1 << "): " << *i1);
    DEBUG_OUT(0,"i2 (" << i2 << "): " << *i2);
    DEBUG_OUT(0,"i3 (" << i3 << "): " << *i3);
    DEBUG_OUT(0,"i1" << (i1==i2?"==":"!=") << "i2");
    DEBUG_OUT(0,"i1" << (i1==i3?"==":"!=") << "i3");
    DEBUG_OUT(0,"i3" << (i3==i2?"==":"!=") << "i2");
}

TEST(SandBox, Inheritance) {
    class A {
    public:
        virtual void f(int) = 0;
        virtual void f(int, int, int) = 0;
    };
    class AA {
    public:
        virtual void f(int, double) {
            DEBUG_OUT(0,"AA::f(int, double)");
        }
    };
    class B: public A, public AA {
    public:
        //========================//
        // Needed for name lookup //
        using AA::f;
        //========================//
        virtual void f(int) override {
            f(0,0.);
        }
        virtual void f(int, int, int) override {
            f(0);
        }
    };
    class C: public B {
    public:
        virtual void f(int, double) {
            DEBUG_OUT(0,"C::f(int, double)");
        }
        virtual void f(int) override {
            f(0,0.);
        }
    };
    A * a = new C();
    a->f(0);
    a->f(0,0,0);
}
