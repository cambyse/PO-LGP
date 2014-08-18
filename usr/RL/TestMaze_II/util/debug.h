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

//#define NDEBUG // to turn of asserts

#include <iostream>
#include <assert.h>
#include "ColorOutput.h"

#ifndef DEBUG_STRING
#define DEBUG_STRING __FILE__ << ": "
#endif

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 0
#endif
#ifndef FORCE_DEBUG_LEVEL
#define FORCE_DEBUG_LEVEL 0
#endif

#define IF_DEBUG(level) if(level<=FORCE_DEBUG_LEVEL || (FORCE_DEBUG_LEVEL==0 && level<=DEBUG_LEVEL))

#define DEBUG_ERROR(message) {                                          \
        std::cerr << ColorOutput::fg_red() << ColorOutput::bold() << "Error(" << __FILE__ << ":" << __LINE__ << "): " << message << ColorOutput::reset_all() << std::endl; \
    }

#define DEBUG_WARNING(message) {                                          \
        std::cerr << ColorOutput::fg_magenta() << "Warning(" << __FILE__ << ":" << __LINE__ << "): " << message << ColorOutput::reset_all() << std::endl; \
    }

#define DEBUG_OUT(level,message) {                                      \
        IF_DEBUG(level) {                                               \
            std::cout << DEBUG_STRING << message << std::endl;          \
        }                                                               \
    }

#define DEBUG_DEAD_LINE {                                       \
        DEBUG_ERROR("This line should never be reached");       \
    }


#endif /* DEBUG_H_ */
