#include <gtest/gtest.h>

#include <memory> // for shared_ptr

#define DEBUG_LEVEL 1
#include "../util/debug.h"

template <class DerivedClass>
class AbstractTemplateClass {
public:
    class PointerType {
    public:
        PointerType(const DerivedClass * d): ptr(d) {}
        virtual operator const DerivedClass() const {
            return ptr.operator*();
        }
        virtual const DerivedClass & operator*() const final {
            return ptr.operator*();
        }
        friend inline std::ostream& operator<<(std::ostream& out, const PointerType& ptr) {
            return out << (DerivedClass)ptr;
        }
    private:
        std::shared_ptr<const DerivedClass> ptr;
    };
};

class AbstractClass: public AbstractTemplateClass<AbstractClass> {
public:
    AbstractClass() {}
    virtual const char * print() const {
        return "AbstractClass";
    }
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractClass& a) {
        return out << a.print();
    }
};

class DerivedClass: public AbstractClass {
public:
    DerivedClass() {}
    virtual const char * print() const override {
        return "DerivedClass";
    }
};

namespace {

    TEST(TemplateTest, TemplateCastOperator) {

        AbstractClass::PointerType ptr(new const DerivedClass());

        DEBUG_OUT(0, ptr);
        DEBUG_OUT(0, *ptr);

        return;
    }
}

