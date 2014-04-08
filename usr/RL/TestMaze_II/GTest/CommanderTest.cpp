#include <gtest/gtest.h>

#include "../util/util.h"
#include "../util/Commander.h"

#define DEBUG_STRING ""
#include "../util/debug.h"

using namespace function_signature;
using namespace Commander;
using util::operator<<;

TEST(Commander, TMP) {

    std::array<int,4> a = {1,2,3,4};
    std::tuple<int,int,int,int> t;
    t = array_to_tuple<decltype(a),decltype(t)>(a);

    DEBUG_OUT(0,"t = " << t);

    //----------------------------------//
    // Want to be able to do this

    int ext = 5;
    CommandCenter c;
    c.add_command(
        {"first-alias","second-alias"},
        [=](int i, double d = 3.3) -> ReturnType {
            if(i<0) {
                return {false, "First argument must be a positive integer"};
            } else {
                DEBUG_OUT(0,"This is a lambda closure using extern variable ext=" << ext <<
                          " plus two arguments i=" << i << " and d=" << d);
                return {true, "I successfully printed a message"};
            }
        },
        "This is a description of the command"
        );
    // DEBUG_OUT(0,c.get_help());
    DEBUG_OUT(0,c.execute("first-alias -2 3.4")); // error
    DEBUG_OUT(0,c.execute("second-alias 5")); // success
    DEBUG_OUT(0,c.execute("inexistent-command")); // error

    //----------------------------------//
}
