/** @file debug_exclude.h
 *
 * \brief Clean up after including debug.h.
 *
 * Include this file at the end of a header to clean up the global name space
 * after having included debug.h.
 *
 */

#ifdef DEBUG_H_

#undef DEBUG_STRING
#undef DEBUG_LEVEL
#undef DEBUG_OUT
#undef DEBUG_ERROR
#undef DEBUG_WARNING
#undef DEBUG_DEAD_LINE

#undef DEBUG_H_ // debug.h may now be re-included

#endif /* DEBUG_H_ */
