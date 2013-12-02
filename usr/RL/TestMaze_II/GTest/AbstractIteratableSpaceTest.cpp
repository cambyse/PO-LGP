#include <gtest/gtest.h>
#include "../util.h"
#include "../debug.h"

namespace {

class TestSpace: public util::AbstractIteratableSpace {
public:
enum ITEM_T { ZERO, ONE, TWO, THREE, END } item;
    TestSpace(ITEM_T i = ZERO): item(i) {}
    virtual Iterator begin() const override { return Iterator(ptr_t(new TestSpace(ZERO))); }
    virtual Iterator end() const override { return Iterator(ptr_t(new TestSpace(END))); }
    virtual ptr_t next() const override { return ptr_t(new TestSpace((ITEM_T)(item+1))); }
    virtual bool operator!=(const AbstractIteratableSpace& other) const override {
        const TestSpace * other_ptr = dynamic_cast<const TestSpace *>(&other);
        return other_ptr==nullptr || this->item!=other_ptr->item;
    }
};

    TEST(AbstractIteratableSpaceTest, Iterate) {

        // construct an object and check constructor
        TestSpace test_space_item(TestSpace::ZERO);
        EXPECT_EQ(TestSpace::ZERO, test_space_item.item) << "constructor FAILED";

        // check next() function
        auto next_test_space_item = std::dynamic_pointer_cast<const TestSpace>(test_space_item.next());
        EXPECT_NE(std::shared_ptr<const TestSpace>(),next_test_space_item) << "cast to derived FAILED";
        EXPECT_EQ(TestSpace::ONE, next_test_space_item->item) << "next() function FAILED";

        int counter = 0;
        for(auto i : TestSpace()) {

            // cast to correct type
            const TestSpace * ii;
            ii = dynamic_cast<const TestSpace *>(i.get());
            EXPECT_NE(ii,nullptr);

            // check value
            switch(counter) {
            case 0:
                EXPECT_EQ(TestSpace::ZERO,ii->item) << "zeroth iteration";
                break;
            case 1:
                EXPECT_EQ(TestSpace::ONE,ii->item) << "first iteration";
                break;
            case 2:
                EXPECT_EQ(TestSpace::TWO,ii->item) << "second iteration";
                break;
            case 3:
                EXPECT_EQ(TestSpace::THREE,ii->item) << "third iteration";
                break;
            default:
                EXPECT_TRUE(false) << "This line should never be reached";
            }
            ++counter;
            if(counter>3) {
                break;
            }
        }
    }

}; // end namespace
