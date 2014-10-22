/*
 * debug_exclude.h
 *
 *  Created on: May 5, 2012
 *      Author: robert
 *
 *      Include this file at the end of a header
 *      to "clean up" the global name space.
 */

#ifdef DEBUG_H_

#undef DEBUG_STRING
#undef DEBUG_LEVEL
#undef DEBUG_OUT

#undef DEBUG_H_ // debug.h may now be re-included

#endif /* DEBUG_H_ */
