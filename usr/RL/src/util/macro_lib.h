#ifndef MACRO_LIB_H_
#define MACRO_LIB_H_

#include <string.h>

/** Like __FILE__ macro but with leading path removed, i.e., only the file name with extension. */
#define FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)


#define HEADER_WARNING(message)                                         \
    static_assert(!((__FILE__[strlen(__FILE__)-1]=='h' && __FILE__[strlen(__FILE__)-2]=='.') || \
                    (__FILE__[strlen(__FILE__)-1]=='p' && __FILE__[strlen(__FILE__)-2]=='p' && \
                     __FILE__[strlen(__FILE__)-3]=='h' && __FILE__[strlen(__FILE__)-4]=='.')), \
                  "This file seems to be a header file (ends in '.h' or '.hpp'). " message);

#define RECURSIVE_EVAL(...)  RECURSIVE_EVAL1(RECURSIVE_EVAL1(RECURSIVE_EVAL1(__VA_ARGS__)))
#define RECURSIVE_EVAL1(...) RECURSIVE_EVAL2(RECURSIVE_EVAL2(RECURSIVE_EVAL2(__VA_ARGS__)))
#define RECURSIVE_EVAL2(...) RECURSIVE_EVAL3(RECURSIVE_EVAL3(RECURSIVE_EVAL3(__VA_ARGS__)))
#define RECURSIVE_EVAL3(...) RECURSIVE_EVAL4(RECURSIVE_EVAL4(RECURSIVE_EVAL4(__VA_ARGS__)))
#define RECURSIVE_EVAL4(...) RECURSIVE_EVAL5(RECURSIVE_EVAL5(RECURSIVE_EVAL5(__VA_ARGS__)))
#define RECURSIVE_EVAL5(...) __VA_ARGS__

#define CONCATENATE(a, ...) PRIMITIVE_CONCATENATE(a, __VA_ARGS__)
#define PRIMITIVE_CONCATENATE(a, ...) a ## __VA_ARGS__

#endif /* MACRO_LIB_H_ */
