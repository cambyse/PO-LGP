#include <gtest/gtest.h>

#include "../util/Commander.h"

#define DEBUG_STRING ""
#include "../util/debug.h"

using namespace function_signature;
using namespace Commander;

template<class From, class To>
class Trafo {
public:
    To operator()(From f) const {
        return To();
    }
};

template<>
class Trafo<QString,int> {
public:
    int operator()(QString from) const {
        //return 11;
        return from.toInt();
    }
};

template<>
class Trafo<QString,double> {
public:
    int operator()(QString from) const {
        //return 2.2;
        return from.toDouble();
    }
};

TEST(Commander, TMP) {

    using namespace std::placeholders;

    std::function<void(int,double)> f = [](int i, double j)->void{
        DEBUG_OUT(0,"i=" << i);
        DEBUG_OUT(0,"j=" << j);
    };

    QString arr[2] = {"1","2"};

    auto ff = recursive_bind<QString,0,Trafo,void,int,double>(arr,f);
    ff();

    return;

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
