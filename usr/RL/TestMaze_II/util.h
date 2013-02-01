/*
 * util.h
 *
 *  Created on: Oct 10, 2012
 *      Author: robert
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <QString>

namespace util {

bool arg_int(const QString& string, const int& n, int& i);
bool arg_double(const QString& string, const int& n, double& d);
bool arg_string(const QString& string, const int& n, QString& s);
template < class C >
C min(const C& c1, const C& c2) { return c1<c2 ? c1 : c2; }
template < class C >
C max(const C& c1, const C& c2) { return c1>c2 ? c1 : c2; }

} // end namespace util

#endif /* UTIL_H_ */
