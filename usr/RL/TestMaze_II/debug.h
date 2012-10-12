/*
 * debug.h
 *
 *  Created on: May 2, 2012
 *      Author: robert
 *
 *      This file should be included in *.cpp files ONLY!
 *      If included in header files include "debug_exclude.h"
 *      at the end of the file to prevent messing up the global
 *      name space.
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <iostream>

#ifndef DEBUG_STRING
	#define DEBUG_STRING __FILE__ << ": "
#endif

#ifndef DEBUG_LEVEL
	#define DEBUG_LEVEL 0
#endif

#define DEBUG_OUT(level,message) { if(level<=DEBUG_LEVEL) { std::cout << DEBUG_STRING << message << std::endl; } }

#endif /* DEBUG_H_ */
