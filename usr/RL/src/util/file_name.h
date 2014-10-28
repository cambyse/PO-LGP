#ifndef FILE_NAME_H_
#define FILE_NAME_H_

#include <stdint.h>

namespace file_name {
    /** Returns the index after the last '/' in path. */
    constexpr int32_t basename_index(const char * const path, const int32_t index = 0, const int32_t slash_index = -1) {
        // Processes 'path' from first to last character. When reaching the end
        // (path is null-terminated) the index of the last '/' plus one is
        // returned. That is, 'slash_index' remembers the index of the last
        // occurrence of '/' in path while 'index' is the current index.
        return path[index] ?                                // is null-character '\0' ?
            (path[index] == '/' ?                           // current char is '/' ?
             basename_index(path, index + 1, index) :       // yes --> recursive call so that slash_index=index and index=index+1
             basename_index(path, index + 1, slash_index)   // no  --> recursive call so that just index=index+1
                ) :
            (slash_index + 1)                              // return index after current slash_index
            ;
    }
}

/** Like __FILE__ macro but with leading path removed, i.e., only the file name with extension. */
#define __FILENAME__ ({                                                 \
            static const int32_t basename_idx = file_name::basename_index(__FILE__); \
            static_assert(basename_idx >= 0, "compile-time basename"); \
            __FILE__ + basename_idx;                                    \
        })

#endif /* FILE_NAME_H_ */
