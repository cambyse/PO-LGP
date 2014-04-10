#include <gtest/gtest.h>

#include "../util/util.h"
#include "../util/Commander.h"

#include <iostream>
#include <string>
#include <unordered_map>
#include <algorithm>

#include "../util/debug.h"

using namespace function_signature;
using namespace Commander;
using util::operator<<;

TEST(Commander, TMP) {

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
        "ext++",
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
    c.add_command({"aaa bbb","blub"}, []()->ReturnType{return{true,"Yeah 1"};}, "multicommand 1");
    c.add_command({"aaa bbb","blub bla"}, []()->ReturnType{return{true,"Yeah 1"};}, "multicommand 1.1");
    c.add_command({"aaa bbb aaa"}, []()->ReturnType{return{true,"Yeah 2"};}, "multicommand 2");
    c.add_command({"aaa ccc aaa"}, []()->ReturnType{return{true,"Yeah 3"};}, "multicommand 3");
    c.add_command({"aaa ccc aaa"}, [](int)->ReturnType{return{true,"Yeah 4"};}, "multicommand 4");
    c.add_command({"aaa ccc aaa","bla"}, [](int,int)->ReturnType{return{true,"Yeah 4.1"};}, "multicommand 4.1");
    c.add_command({"aaa ccc bbb"}, []()->ReturnType{return{true,"Yeah 5"};}, "multicommand 5");
    c.add_command({"aaa ccc bbb ddd"}, []()->ReturnType{return{true,"Yeah 6"};}, "multicommand 6");
    c.add_command({"aaa ccc bbb eee"}, []()->ReturnType{return{true,"Yeah 7"};}, "multicommand 7");
    c.add_command({"bbb ccc ddd"}, []()->ReturnType{return{true,"Yeah 8"};}, "multicommand 8");
    c.add_command({"bbb ccc ddd eee"}, []()->ReturnType{return{true,"Yeah 9"};}, "multicommand 9");
    c.add_command("duplicate", []()->ReturnType{return{true,""};}, "Same command name same signature");
    c.add_command("duplicate", []()->ReturnType{return{true,""};}, "Same command name same signature");
    c.add_command("duplicate", [](int)->ReturnType{return{true,""};}, "Same command name different signature");
    c.add_command("duplicate", [](double)->ReturnType{return{true,""};}, "Same command name different signature (but redundant)");
    c.add_command({"help","h"}, [&]()->ReturnType{DEBUG_OUT(0,"\n\n" << c.get_help_string() << "\n");return{true,""};}, "Print help");

    DEBUG_OUT(0, c.execute("help"));
    DEBUG_OUT(0, c.execute("aaa bbb"));
    DEBUG_OUT(0, c.execute("aaa bbb aaa"));
    DEBUG_OUT(0, c.execute("aaa ccc aaa"));
    DEBUG_OUT(0, c.execute("aaa ccc aaa 1000"));
    DEBUG_OUT(0, c.execute("aaa ccc bbb"));
    DEBUG_OUT(0, c.execute("aaa ccc bbb ddd"));
    DEBUG_OUT(0, c.execute("aaa ccc bbb eee"));
    DEBUG_OUT(0, c.execute("bbb ccc ddd"));
    DEBUG_OUT(0, c.execute("bbb ccc ddd eee"));
    // DEBUG_OUT(0,"Returned Message: " << c.execute("-->n 10"));                  // ok
    // DEBUG_OUT(0,"Returned Message: " << c.execute("ext++"));                    // increment ext
    // DEBUG_OUT(0,"Returned Message: " << c.execute("-->n 10"));                  // ok
    // DEBUG_OUT(0,"Returned Message: " << c.execute("-->n not-a-number"));        // error (cannot be interpreted as int)
    // DEBUG_OUT(0,"Returned Message: " << c.execute("-->n"));                     // error (not enough arguments)
    // DEBUG_OUT(0,"Returned Message: " << c.execute("-->n 1 2 3"));               // error (too many arguments)
    // DEBUG_OUT(0,"Returned Message: " << c.execute("-->n -3"));                  // error (arg must be non-negative)
    // DEBUG_OUT(0,"Returned Message: " << c.execute("inexistent-command"));       // error
    // DEBUG_OUT(0,"Returned Message: " << c.execute("-->n 10; ext++; -->n 10"));  // multiple commands

    //------------------------------------------------------------//
}
