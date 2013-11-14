#include <devTools/logging.h>

// set your logger wiht SET_LOG(<NAME>, <LEVEL>), a name is just a name for a
// logger. This is handy, if you want to log only a part of the whole system.
// Level is either DEBUG, INFO, WARN or ERROR. Any message above the current
// verbosity level is not logged.

SET_LOG(main, DEBUG);

int main() {
  // The easiest way to log is by putting some string to the DEBUG, INFO, WARN
  // or ERROR macro. First give the name of the logger, then the message.
  DEBUG(main, "Some debug text.");

  // You can also directly output a variable with its value. Every type, that
  // supports operator<< can be used here. This works for DEBUG_VAR andd
  // INFO_VAR.
  int x=6;
  INFO_VAR(main, x);

  // For more generic messages a variadic macro is defined. You can concatenate
  // every type, that supports operator<< with that.
  std::string str = "variadic";
  WARN(main, "this is a ", str, " macro.");

  // ERROR messanges are printed to std::cerr instead of std::cout. They don't
  // halt the programm.
  ERROR(main, "An error message");
}
