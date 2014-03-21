#include <gtest/gtest.h>

#include "SandBox.h"

#include <memory> // for shared_ptr

#include "../util/debug.h"

using std::shared_ptr;

Notifier::Notifier(): to_be_notified(nullptr) {}
Notifier::~Notifier() { to_be_notified->notify_me(); }
void Notifier::set_to_be_notified(A * p) { to_be_notified = p; }

A::A(): notifier(new Notifier()) {}
A::~A() { delete notifier; }
void A::set_to_be_notified(A * p) const { notifier->set_to_be_notified(p); }
void A::notify_me() {
    got_notified = true;
    DEBUG_OUT(0,"I got notified");
}

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

TEST(SandBox, ConstPtrMember) {
    // Problem: How can I "subscribe" to a const object A to be notified?
    // Solution: The const object A holds a pointer to a Notifyer object. In
    // that way A remains unchanged and only its Notifyer object "remembers" to
    // other object to be notified.
    A * a_ptr = new A();
    {
        const A a;
        a.set_to_be_notified(a_ptr);
    }
}
