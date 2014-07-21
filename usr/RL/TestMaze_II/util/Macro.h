#ifndef UTIL_MACROS_H_
#define UTIL_MACROS_H_

#define COMPARE_ABSTRACT_TYPE_AND_CAST(comp,type_getter,type_for_cast)  \
    auto ptr = dynamic_cast<type_for_cast>(&other);                     \
    if(this->type_getter()!=other.type_getter()) {                      \
        return this->type_getter() comp other.type_getter();            \
    }                                                                   \
    if(ptr==nullptr) {                                                  \
        DEBUG_ERROR("Dynamic cast failed");                             \
        return true;                                                    \
    }

#endif /* UTIL_MACROS_H_ */
