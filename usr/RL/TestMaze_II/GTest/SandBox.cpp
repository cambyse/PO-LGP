#include <gtest/gtest.h>

#include <memory> // for shared_ptr

#include "../debug.h"

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
