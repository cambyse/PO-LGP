#include <gtest/gtest.h>

#include "../util/Commander.h"

#define DEBUG_STRING ""
#include "../util/debug.h"

TEST(Commander, TMP) {

    typedef Commander::CommandList command_list_t;
    typedef Commander::Description description_t;

    int i = 5;

    Commander c;
    c.add(command_list_t("do_nothing"),
          Commander::Arguments<>(),
          Commander::Arguments<>(),
          description_t("Prints \"I don't do anything!\""),
          [](){DEBUG_OUT(0,"I don't do anything!");}
        );
    c.add(command_list_t("print_i"),
          Commander::Arguments<>(),
          Commander::Arguments<>(),
          description_t("Prints variable i from CommanderTest body."),
          [&](){DEBUG_OUT(0,"i = " << i);}
        );

    auto com = c.print_commands();
    DEBUG_OUT(0,"");
    for(auto line : com) {
        DEBUG_OUT(0,line);
    }
    DEBUG_OUT(0,"");

    c.execute("do_nothing");
    c.execute("print_i");

    // auto func1 = [] () { DEBUG_OUT(0,"empty lambda"); };
    // auto func2 = [=] () { DEBUG_OUT(0,"print i: " << i); };
    // func2();
    // func1();
}
