#include <iostream>
#include <QString>
#include <string>
#include "../util/QtUtil.h"

#include "../util/Commander.h"

using namespace std;
using namespace Commander;

// typedef return type for convenience
typedef ReturnType R;

class SomeClass {
public:
    void define_commands(CommandCenter & c) const {
        c.add_command("f", [this]()->R {
                f();
                return {true, ""};
            }, "call private function f() in SomeClass");
    }
private:
    void f() {
        cout << "Calling f()" << endl;
    }
};

int main(int,char**) {

    //------------------//
    // setup everything //
    //------------------//

    // lines read from cin
    string line;

    // whether to quit the application
    bool quit = false;

    // some class
    SomeClass some_class;

    // the command center
    CommandCenter c;

    //----------------------//
    // define some commands //
    //----------------------//

    // simple example
    c.add_command("simple", []()->R{
            cout << "simple function call" << endl;
            return {true,""};
        },"call a simple function");

    // help
    c.add_command({"help","h"}, [&c]()->R{
            cout << c.get_help_string() << endl;
            return {true, ""};
        },"print help");

    // quit
    c.add_command({"quit","q"}, [&quit]()->R{
            quit=true;
            cout << "quit application" << endl;
            return {true, ""};
        },"print help");

    // handing over to class
    some_class.define_commands(c);

    // groups of commands
    c.add_command("take double", [](double d)->R{
            cout << "Got double " << d << endl;
            return {true, ""};
        },"print <double>");

    c.add_command({"take int","iii"}, [](int i)->R{
            cout << "Got int " << i << endl;
            return {true, ""};
        },"print <int>");

    c.add_command("take bool", [](bool b)->R{
            cout << "Got bool " << b << endl;
            return {true, ""};
        },"print <bool>");

    // overloading
    c.add_command("overload", [](int)->R{
            return {true, ""};
        },"takes <int>");
    c.add_command("overload", [](double)->R{
            return {true, ""};
        },"takes <double>");

    // rejecting based on value
    c.add_command("positive int", [](int i)->R{
            if(i>=0) {
                return {true, QString("%1 is positive").arg(i)};
            } else {
                return {false, QString("%1 is negative").arg(i)};
            }
        },"accepts only positive value for <int>");

    // topics
    c.add_command("Topic 1","com1", []()->R{return {true, ""};},"");
    c.add_command("Topic 1","com2", []()->R{return {true, ""};},"");
    c.add_command("New Topic 2","com3", []()->R{return {true, ""};},"");
    c.add_command({1,"Another Topic 3"},"com4", []()->R{return {true, ""};},"");
    c.add_command({0.5,"Yet Another Topic 4"},"com5", []()->R{return {true, ""};},"");

    // pitfalls with lamda closures
    {
        double local_variable = 10;
        c.add_command("local var 1", [&local_variable]()->R{
                cout << "local variable is " << local_variable << endl;
                return {true, ""};
            },"ref to (local) var (undefined behavior)");
        local_variable = 11;
        c.add_command("local var 2", [local_variable]()->R{
                cout << "local variable is " << local_variable << endl;
                return {true, ""};
            },"copy of (local) var");
        local_variable = 12;
    }
    double * dPtr = new double();
    c.add_command("seg", [dPtr]()->R{
            *dPtr = 10;
            cout << "set *dPtr to " << *dPtr << endl;
            return {true, ""};
        },"produces a segfault");
    delete dPtr;

    // identically named commands
    c.add_command("iden", []()->R{return {true, ""};},"identically named");
    c.add_command("iden", []()->R{return {true, ""};},"identically named");

    // unknown types
    // class unknown_type {};
    // c.add_command("unknown type", [](unknown_type)->R{
    //         return {true, ""};
    //     },"takes <unknown_type>");

    //------------------------------------------//
    // read commands and pass to command center //
    //------------------------------------------//

    while(getline(cin,line)) {

        // print answer
        cout << "CommandCenter answers: " << c.execute(QString(line.c_str())) << endl;

        // quit application
        if(quit) {
            return 0;
        }
    }

    return 0;
}
