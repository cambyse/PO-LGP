/** @file debug.h
 *
 * \brief Provide simple debuggin output with adaptive verbosity level.
 *
 * This file should be included in *.cpp files ONLY! If included in header files
 * include debug_exclude.h at the end of the file to prevent messing up the
 * global name space.
 *
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <iostream>
#include "./util/ColorOutput.h"

#ifndef DEBUG_STRING
#define DEBUG_STRING __FILE__ << ": "
#endif

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 0
#endif

#define DEBUG_ERROR(message) {                                          \
        std::cout << ColorOutput::fg_red() << DEBUG_STRING << "Error(L" << __LINE__ << "): " << message << ColorOutput::reset_all() << std::endl; \
    }

#define DEBUG_OUT(level,message) {                              \
        if(level<=DEBUG_LEVEL) {                                \
            std::cout << DEBUG_STRING << message << std::endl;  \
        }                                                       \
    }

#define DEBUG_DEAD_LINE {                               \
        DEBUG_ERROR("This line (" << __LINE__ <<        \
                    ") in file " << __FILE__ <<         \
                    " should never be reached");        \
    }

#define DEBUG_IF(check) if(DEBUG_LEVEL>0 && (check))

#endif /* DEBUG_H_ */
