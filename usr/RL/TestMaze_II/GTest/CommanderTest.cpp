#include <gtest/gtest.h>

#include "../util/util.h"
#include "../util/Commander.h"

#include "../util/debug.h"

using namespace function_signature;
using namespace Commander;
using util::operator<<;

TEST(Commander, TMP) {

    //----------------------------------//
    // Want to be able to do this

    int ext = 3;
    CommandCenter c;
    c.add_command(
        {"count-until-n","-->n"},
        [&](int n) -> ReturnType {
            if(n<0) {
                return {false, "First argument must be a non-negative int"};
            }
            DEBUG_OUT(0,">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            DEBUG_OUT(0,"External starting value: " << ext);
            for(int i=ext; i<=n; ++i) {
                DEBUG_OUT(0, i);
            }
            DEBUG_OUT(0,"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            return {true, QString("Counted from %1 to %2").arg(ext).arg(n)};
        },
        "Counts from an externaly given value up until first argument"
        );
    c.add_command(
        {"ext++"},
        [&]() -> ReturnType {
            DEBUG_OUT(0,">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            DEBUG_OUT(0,"ext " << ext << " --> " << ext+1);
            DEBUG_OUT(0,"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            ++ext;
            return {true, "Incremented"};
        },
        "Increments external value from command '-->n'"
        );
    //c.add_command({"wrong"}, [](){}, "Wrong return type");
    c.add_command({"duplicate"}, []()->ReturnType{return{true,""};}, "Same command name same signature");
    c.add_command({"duplicate"}, []()->ReturnType{return{true,""};}, "Same command name same signature");
    c.add_command({"duplicate"}, [](int)->ReturnType{return{true,""};}, "Same command name different signature");


    DEBUG_OUT(0,"\n\n" << c.get_help(5) << "\n");
    DEBUG_OUT(0,"Returned Message: " << c.execute("-->n 10"));              // ok
    DEBUG_OUT(0,"Returned Message: " << c.execute("ext++"));                // increment ext
    DEBUG_OUT(0,"Returned Message: " << c.execute("-->n 10"));              // ok
    DEBUG_OUT(0,"Returned Message: " << c.execute("-->n not-a-number"));    // error (cannot be interpreted as int)
    DEBUG_OUT(0,"Returned Message: " << c.execute("-->n"));                 // error (not enough arguments)
    DEBUG_OUT(0,"Returned Message: " << c.execute("-->n -3"));              // error (arg must be non-negative)
    DEBUG_OUT(0,"Returned Message: " << c.execute("inexistent-command"));   // error

    //----------------------------------//
}
