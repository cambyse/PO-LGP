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



/// Macro that expands to the number of arguments given. This is taken from
/// http://efesx.com/2010/07/17/variadic-macro-to-count-number-of-arguments/
#define VA_ARGS_NUM(...) VA_ARGS_NUM_IMPL(__VA_ARGS__, 50,49,48,47,46,45,44,43,42,41,40, \
                                          39,38,37,36,35,34,33,32,31,30, \
                                          29,28,27,26,25,24,23,22,21,20, \
                                          19,18,17,16,15,14,13,12,11,10, \
                                          9,8,7,6,5,4,3,2,1)
#define VA_ARGS_NUM_IMPL(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,                \
                         _11,_12,_13,_14,_15,_16,_17,_18,_19,_20,       \
                         _21,_22,_23,_24,_25,_26,_27,_28,_29,_30,       \
                         _31,_32,_33,_34,_35,_36,_37,_38,_39,_40,       \
                         _41,_42,_43,_44,_45,_46,_47,_48,_49,_50,N,...) N
#define VA_ARGS_NUM_DIV_2(...) VA_ARGS_NUM_IMPL(__VA_ARGS__, 25,24,24,23,23,22,22,21,21,20,20, \
                                                19,19,18,18,17,17,16,16,15,15,14,14,13,13,12,12,11,11,10,10, \
                                                9,9,8,8,7,7,6,6,5,5,4,4,3,3,2,2,1,1,0)

#endif /* MACRO_LIB_H_ */
