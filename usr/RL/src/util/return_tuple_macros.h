#include <tuple>

#include <util/macro_lib.h>

// named version
#define TT1(name, t1, v1) t1 v1;                 \
    std::tuple<t1&> name(v1); name
#define TT2(name, t1, v1, t2, v2) t1 v1; t2 v2;         \
    std::tuple<t1&,t2&> name(v1, v2); name
#define TT3(name, t1, v1, t2, v2, t3, v3) t1 v1; t2 v2; t3 v3;         \
    std::tuple<t1&,t2&,t3&> name(v1, v2, v3); name
#define TT4(name, t1, v1, t2, v2, t3, v3, t4, v4) t1 v1; t2 v2; t3 v3; t4 v4;         \
    std::tuple<t1&,t2&,t3&,t4&> name(v1, v2, v3, v4); name
#define TT5(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5) t1 v1; t2 v2; t3 v3; t4 v4; t5 v5;         \
    std::tuple<t1&,t2&,t3&,t4&,t5&> name(v1, v2, v3, v4, v5); name
#define TT6(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6) t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&> name(v1, v2, v3, v4, v5, v6); name
#define TT7(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7) t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&> name(v1, v2, v3, v4, v5, v6, v7); name
#define TT8(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8) t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8; \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&> name(v1, v2, v3, v4, v5, v6, v7, v8); name
#define TT9(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9) t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8; t9 v9; \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&,t9&> name(v1, v2, v3, v4, v5, v6, v7, v8, v9); name
#define TT10(name, t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9, t10, v10) t1 v1; t2 v2; t3 v3; t4 v4; t5 v5; t6 v6; t7 v7; t8 v8; t9 v9; t10 v10; \
    std::tuple<t1&,t2&,t3&,t4&,t5&,t6&,t7&,t8&,t9&,t10&> name(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10); name

// anonymous version
#define T1(t1, v1)                                               \
    TT1(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1)
#define T2(t1, v1, t2, v2)                                               \
    TT2(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2)
#define T3(t1, v1, t2, v2, t3, v3)                                       \
    TT3(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2, t3, v3)
#define T4(t1, v1, t2, v2, t3, v3, t4, v4)                               \
    TT4(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2, t3, v3, t4, v4)
#define T5(t1, v1, t2, v2, t3, v3, t4, v4, t5, v5)                       \
    TT5(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2, t3, v3, t4, v4, t5, v5)
#define T6(t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6)               \
    TT6(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6)
#define T7(t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7)       \
    TT7(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7)
#define T8(t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8) \
    TT8(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8)
#define T9(t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9) \
    TT9(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9)
#define T10(t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9, t10, v10) \
    TT10(CONCATENATE(unique_tuple_identifyer_,__LINE__), t1, v1, t2, v2, t3, v3, t4, v4, t5, v5, t6, v6, t7, v7, t8, v8, t9, v9, t10, v10)
