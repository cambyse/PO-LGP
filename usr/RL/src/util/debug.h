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
#include "macro_lib.h"

#ifndef DEBUG_STRING
#define DEBUG_STRING FILENAME << ": "
#endif

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 0
#endif
#ifndef FORCE_DEBUG_LEVEL
#define FORCE_DEBUG_LEVEL 0
#endif

#define IF_DEBUG(level) if(level<=FORCE_DEBUG_LEVEL || (FORCE_DEBUG_LEVEL==0 && level<=DEBUG_LEVEL))

#define DEBUG_ERROR(message) {                                          \
        std::cerr << ColorOutput::fg_red() << ColorOutput::bold() << "Error(" << FILENAME << ":" << __LINE__ << "): " << message << ColorOutput::reset_all() << std::endl; \
    }

#define DEBUG_WARNING(message) {                                          \
        std::cerr << ColorOutput::fg_magenta() << "Warning(" << FILENAME << ":" << __LINE__ << "): " << message << ColorOutput::reset_all() << std::endl; \
    }

#define DEBUG_OUT(level,message) {                                      \
        IF_DEBUG(level) {                                               \
            std::cout << DEBUG_STRING << message << std::endl;          \
        }                                                               \
    }

#define DEBUG_DEAD_LINE {                                       \
        DEBUG_ERROR("This line should never be reached");       \
    }

#define DEBUG_EXPECT(level, condition) {                \
        IF_DEBUG(level) {                               \
            if(!(condition)) {                          \
                DEBUG_ERROR("Condition '" << #condition << "' is not fulfilled"); \
            }                                           \
        }                                               \
    }

#define DEBUG_EXPECT_APPROX(level, value_1, value_2) {  \
        IF_DEBUG(level) {                               \
            if(fabs(value_1-value_2)>1e-10) {                     \
                DEBUG_ERROR("Not approximately equal ('" << #value_1 << "' and '" << #value_2 << "')"); \
                DEBUG_ERROR("    " << #value_1 << " = " << value_1);     \
                DEBUG_ERROR("    " << #value_2 << " = " << value_2);     \
            }                                                           \
        }                                                               \
    }

// assert of errors and warnings for unit tests

#define assert_error(call, msg) {                                       \
        util::GrabStream stream(std::cerr);                             \
        {call;}                                                         \
        EXPECT_EQ("[31m[1m" msg "[0m\n", stream.get_text());      \
    }

#define assert_warning(call, msg) {                             \
        util::GrabStream stream(std::cerr);                     \
        {call;}                                                 \
        EXPECT_EQ("[35m" msg "[0m\n", stream.get_text());   \
    }
#else
static_assert(false, "Including this file multiple times is usually not a good idea. Include 'debug_exclude.h' to exclude.");
#endif /* DEBUG_H_ */
