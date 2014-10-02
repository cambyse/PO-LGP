#ifndef QTUTIL_H
#define QTUTIL_H

#include <QtCore>
#include <ostream>
#include <iostream>

#if QT_VERSION < QT_VERSION_CHECK(4, 9, 9)
#define MY_QT_STR QString::fromUtf8
#else
#define MY_QT_STR QStringLiteral
#endif

class QString;
std::ostream& operator<<(std::ostream &out, const QString& s);

//#define ERROR(x) {std::cout << x << std::endl;}
#define ERROR(x)

//#define DEBUG_OUT(x) {std::cout << x << std::endl;}
#define DEBUG_OUT(x)

#endif // QTUTIL_H
